/**
 * 设备文件系统（LittleFS）辅助：/api/files 的列目录与删除。
 *
 * 目前只用于清理构建产物目录（/web、/sound）里的旧文件；
 * /cards.json 是设备运行时数据，固件端会拒绝删除，这里也一并排除。
 */

export interface DeviceDirEntry {
  name: string
  isDirectory: boolean
}

/** 设备端受保护、任何情况下都不删除的文件（与固件 isProtectedRuntimeFile 一致） */
export const PROTECTED_DEVICE_PATHS = new Set(['/cards.json'])

/** 目录 + 条目名拼成设备完整路径（固件回传的 name 可能已经带目录前缀） */
export function joinDevicePath(directory: string, name: string): string {
  if (name.startsWith('/')) return name
  return directory === '/' ? `/${name}` : `${directory}/${name}`
}

/** 取条目名（去掉固件可能带上的目录前缀） */
export function deviceEntryName(name: string): string {
  const slash = name.lastIndexOf('/')
  return slash >= 0 ? name.slice(slash + 1) : name
}

/** 读取设备目录（固件返回裸 JSON：{path, files:[{name,isDirectory,size}]}） */
export async function listDeviceDirectory(path: string): Promise<DeviceDirEntry[]> {
  const response = await fetch(`/api/files?path=${encodeURIComponent(path)}`)
  if (!response.ok) {
    throw new Error(`读取目录 ${path} 失败 (HTTP ${response.status})`)
  }
  const body = await response.json()
  return Array.isArray(body?.files) ? body.files : []
}

/** 删除设备上的文件或空目录，失败时抛出设备返回的原因 */
export async function deleteDevicePath(path: string): Promise<void> {
  const response = await fetch(`/api/files${encodeURI(path)}`, { method: 'DELETE' })
  if (response.ok) return

  const message = await response
    .json()
    .then((body: any) => body?.message)
    .catch(() => null)
  throw new Error(message || `删除 ${path} 失败 (HTTP ${response.status})`)
}

/* ==================== 旧构建产物自检 ==================== */

/** 构建产物名形如 index-CmaeKLnm.js / style-CB3BRIgW.css（-<内容哈希> 由 Vite 生成） */
const HASHED_ASSET_RE = /^(.+)-([A-Za-z0-9_-]{6,})\.([a-z0-9]+)$/
const WEB_ASSETS_DIR = '/web/assets'

/** 去掉 .gz 后取“前缀 + 扩展名”，例如 index-B7vezpgC.js.gz → index.js */
function hashedAssetKey(name: string): string | null {
  const stem = name.endsWith('.gz') ? name.slice(0, -'.gz'.length) : name
  const match = HASHED_ASSET_RE.exec(stem)
  return match ? `${match[1]}.${match[3]}` : null
}

/** 当前页面真正引用的 /assets/ 资源（index.html 里写的就是设备上应当有的那份） */
function referencedAssetNames(): string[] {
  // vite dev 下资源来自 dev server，设备上根本没有，不能据此删文件
  if (!import.meta.env.PROD) return []

  const names = new Set<string>()
  const collect = (url: string | null | undefined) => {
    const match = /^\/assets\/([^/?#]+)$/.exec(url || '')
    const name = match?.[1]
    if (name) names.add(name)
  }
  document.querySelectorAll('script[src]').forEach(el => collect(el.getAttribute('src')))
  document.querySelectorAll('link[href]').forEach(el => collect(el.getAttribute('href')))
  return [...names]
}

/**
 * 清理设备 /web/assets 里上一次构建残留的旧产物（例如 index-B7vezpgC.js.gz）。
 *
 * 设备端升级时只会上传新文件，旧哈希名的文件不会自己消失；这份自检让页面在
 * 旧版网页做过升级（那次升级还没带清理逻辑）之后也能把垃圾收干净。
 *
 * 两道保险，避免误删：
 *   1. 本页引用的资源必须都能在设备上找到，否则说明页面与设备内容不一致（例如
 *      浏览器拿的是缓存的旧页面），直接放弃；
 *   2. 只删“同前缀 + 同扩展名、内容哈希不同”的文件，其它资源（动态分包等）不碰。
 */
export async function sweepStaleWebAssets(): Promise<string[]> {
  const referenced = referencedAssetNames()
  if (referenced.length === 0) return []

  let entries: DeviceDirEntry[]
  try {
    entries = await listDeviceDirectory(WEB_ASSETS_DIR)
  } catch {
    return [] // 目录不存在或接口不可用：什么都不做
  }

  const files = entries.filter(entry => !entry.isDirectory)
  const deviceNames = new Set(files.map(entry => deviceEntryName(entry.name)))

  const liveNames = new Set<string>()
  for (const name of referenced) {
    liveNames.add(name)
    liveNames.add(`${name}.gz`)
  }
  if (referenced.some(name => !deviceNames.has(name) && !deviceNames.has(`${name}.gz`))) {
    return []
  }

  const liveKeys = new Set(
    referenced.map(hashedAssetKey).filter((key): key is string => key !== null)
  )

  const removed: string[] = []
  for (const entry of files) {
    const name = deviceEntryName(entry.name)
    if (liveNames.has(name)) continue

    const key = hashedAssetKey(name)
    if (key === null || !liveKeys.has(key)) continue

    const path = joinDevicePath(WEB_ASSETS_DIR, entry.name)
    if (PROTECTED_DEVICE_PATHS.has(path)) continue

    try {
      await deleteDevicePath(path)
      removed.push(path)
    } catch {
      /* 删不掉就留着，下次页面加载再试 */
    }
  }
  return removed
}
