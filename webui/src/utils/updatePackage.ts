/**
 * 标准 OTA 升级包（pack_ota.py 产出的 update-package.zip）的解析与上传。
 *
 * 升级包结构：
 *   firmware.bin            固件（写入 OTA 分区）
 *   data/web/*.gz           网页资源
 *   data/sound/*.aac        提示音
 *
 * 设备端不接受 zip：由浏览器解包，再分别上传——
 * 数据文件走 /api/files/upload，固件走 /update（设备收到固件后 1 秒自动重启，
 * 所以固件必须最后上传）。
 */

import { readZipEntries } from './zip'
import { sha256Hex } from './sha256'

/** 升级包里不应写入设备的文件（设备本地数据） */
const EXCLUDED_DATA_FILES = new Set(['cards.json'])

export interface DataFile {
  /** 设备上的目标目录，例如 /web */
  path: string
  /** 文件名，例如 index.html.gz */
  name: string
  /** 未压缩大小 */
  size: number
  read: () => Promise<Uint8Array>
  /** 内容 SHA-256（十六进制），hashPackage() 之后可用 */
  sha256?: string
}

export interface PackageFirmware {
  name: string
  size: number
  read: () => Promise<Uint8Array>
  sha256?: string
}

export interface UpdatePackage {
  firmware: PackageFirmware
  dataFiles: DataFile[]
  /** 数据文件未压缩总大小 */
  dataTotalSize: number
}

/** 设备上的完整路径，例如 /web/index.html.gz */
export function devicePath(file: DataFile): string {
  return `${file.path === '/' ? '' : file.path}/${file.name}`
}

/** 设备对单个文件的处理决定 */
export type SyncAction = 'skip' | 'upload' | 'preserve' | 'invalid'

/** 需要上传的文件 + 被跳过的数量 */
export interface UploadPlan {
  pending: DataFile[]
  skipped: number
  pendingBytes: number
}

/** 为升级包内所有文件计算 SHA-256（浏览器只能自己算，见 sha256.ts） */
export async function hashPackage(pkg: UpdatePackage): Promise<void> {
  for (const file of pkg.dataFiles) {
    file.sha256 = sha256Hex(await file.read())
  }
  pkg.firmware.sha256 = sha256Hex(await pkg.firmware.read())
}

/**
 * 让设备逐个比对自己已有的文件，返回每个文件的动作。
 * 设备端 computeLittleFsFileSha256() 与包内哈希一致时返回 skip。
 */
export async function syncCheck(files: DataFile[]): Promise<Map<string, SyncAction>> {
  const response = await fetch('/api/files/sync-check', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      files: files.map(file => ({ path: devicePath(file), sha256: file.sha256 }))
    })
  })

  if (!response.ok) {
    throw new Error(`比对文件失败 (HTTP ${response.status})`)
  }

  const body = await response.json()
  const results: Array<{ path: string; action: SyncAction }> = body?.data?.results ?? []
  const actions = new Map<string, SyncAction>()
  for (const item of results) {
    actions.set(item.path, item.action)
  }
  return actions
}

/** 根据设备的比对结果算出真正需要上传的文件 */
export function buildUploadPlan(pkg: UpdatePackage, actions: Map<string, SyncAction>): UploadPlan {
  const pending = pkg.dataFiles.filter(file => {
    const action = actions.get(devicePath(file))
    // 只有明确 skip/preserve 才跳过；拿不到结果的按需上传处理
    return action !== 'skip' && action !== 'preserve'
  })

  return {
    pending,
    skipped: pkg.dataFiles.length - pending.length,
    pendingBytes: pending.reduce((sum, file) => sum + file.size, 0)
  }
}

export class UploadCancelled extends Error {
  constructor() {
    super('已取消')
    this.name = 'UploadCancelled'
  }
}

/** 解析升级包，校验必须含 firmware.bin。 */
export async function parseUpdatePackage(file: Blob): Promise<UpdatePackage> {
  const entries = await readZipEntries(file)

  const firmwareEntry =
    entries.find(entry => entry.name === 'firmware.bin') ??
    entries.find(entry => entry.name.endsWith('/firmware.bin'))

  if (!firmwareEntry) {
    throw new Error('升级包缺少 firmware.bin，请使用 pack_ota.py 生成的标准升级包')
  }

  const dataFiles: DataFile[] = []
  for (const entry of entries) {
    if (!entry.name.startsWith('data/')) continue
    const relative = entry.name.slice('data/'.length)
    if (!relative || EXCLUDED_DATA_FILES.has(relative)) continue

    const lastSlash = relative.lastIndexOf('/')
    dataFiles.push({
      path: lastSlash >= 0 ? `/${relative.slice(0, lastSlash)}` : '/',
      name: lastSlash >= 0 ? relative.slice(lastSlash + 1) : relative,
      size: entry.size,
      read: entry.read
    })
  }

  if (firmwareEntry.size === 0) {
    throw new Error('升级包中的 firmware.bin 为空')
  }

  return {
    firmware: { name: 'firmware.bin', size: firmwareEntry.size, read: firmwareEntry.read },
    dataFiles,
    dataTotalSize: dataFiles.reduce((sum, item) => sum + item.size, 0)
  }
}

/** 把响应体里的错误信息提取出来（后端成功响应是 JSON 信封，OTA 错误是纯文本）。 */
function describeError(xhr: XMLHttpRequest): string {
  const text = xhr.responseText || ''
  try {
    const body = JSON.parse(text)
    if (body && typeof body.message === 'string' && body.message) return body.message
  } catch {
    /* 不是 JSON，按纯文本处理 */
  }
  if (text) return text.slice(0, 200)
  return `HTTP ${xhr.status}`
}

interface UploadOptions {
  onProgress?: (loaded: number, total: number) => void
  /** 返回 true 表示已取消，上传会被中断 */
  isCancelled?: () => boolean
  /** 额外请求头，例如 X-File-SHA256 */
  headers?: Record<string, string>
}

function postForm(
  url: string,
  fields: Array<[string, string | Blob]>,
  options: UploadOptions = {}
): Promise<void> {
  return new Promise((resolve, reject) => {
    const formData = new FormData()
    for (const [key, value] of fields) {
      formData.append(key, value)
    }

    const xhr = new XMLHttpRequest()
    let settled = false

    const finish = (fn: () => void) => {
      if (settled) return
      settled = true
      fn()
    }

    if (options.onProgress) {
      xhr.upload.addEventListener('progress', event => {
        if (event.lengthComputable) {
          options.onProgress?.(event.loaded, event.total)
        }
      })
    }

    xhr.addEventListener('load', () =>
      finish(() => {
        if (xhr.status === 200) resolve()
        else reject(new Error(describeError(xhr)))
      })
    )
    xhr.addEventListener('error', () => finish(() => reject(new Error('网络错误，请检查连接'))))
    xhr.addEventListener('abort', () => finish(() => reject(new UploadCancelled())))
    xhr.addEventListener('timeout', () => finish(() => reject(new Error('上传超时'))))

    xhr.timeout = 180000
    xhr.open('POST', url)

    if (options.headers) {
      for (const [name, value] of Object.entries(options.headers)) {
        xhr.setRequestHeader(name, value)
      }
    }

    if (options.isCancelled?.()) {
      reject(new UploadCancelled())
      return
    }
    xhr.send(formData)

    // 取消时中断当前请求
    if (options.isCancelled) {
      const timer = window.setInterval(() => {
        if (options.isCancelled?.()) {
          window.clearInterval(timer)
          xhr.abort()
        }
      }, 200)
      xhr.addEventListener('loadend', () => window.clearInterval(timer))
    }
  })
}

/** 上传一个数据文件到设备（path 字段必须排在 file 之前，固件在首个数据块就读取目标目录）。 */
export function uploadDataFile(file: DataFile, options: UploadOptions = {}): Promise<void> {
  return file.read().then(bytes =>
    postForm(
      '/api/files/upload',
      [
        ['path', file.path],
        ['file', new File([bytes as BlobPart], file.name)]
      ],
      {
        ...options,
        headers: { ...options.headers, ...(file.sha256 ? { 'X-File-SHA256': file.sha256 } : {}) }
      }
    )
  )
}

/** 上传固件（设备写完后 1 秒自动重启，务必放在最后一步）。 */
export function uploadFirmware(
  firmware: PackageFirmware,
  options: UploadOptions = {}
): Promise<void> {
  return firmware.read().then(bytes =>
    postForm('/update', [['firmware', new File([bytes as BlobPart], firmware.name)]], {
      ...options,
      headers: { ...options.headers, ...(firmware.sha256 ? { 'X-Firmware-SHA256': firmware.sha256 } : {}) }
    })
  )
}
