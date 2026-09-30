/**
 * 极简 zip 读取器（只读，够解析 OTA 升级包用）。
 *
 * 不引入 fflate 之类的库：解压交给浏览器内置的 DecompressionStream('deflate-raw')，
 * 这里只负责解析 zip 的中央目录与局部头。整包 1MB 左右，按需解压，不常驻内存。
 */

const EOCD_SIGNATURE = 0x06054b50
const CENTRAL_SIGNATURE = 0x02014b50
const LOCAL_SIGNATURE = 0x04034b50

export interface ZipEntry {
  /** zip 内的完整路径，例如 data/web/index.html.gz */
  name: string
  /** 未压缩大小（字节） */
  size: number
  /** 按需解压：返回该条目的内容 */
  read: () => Promise<Uint8Array>
}

/** 从尾部向前找 End of Central Directory（注释最长 65535 字节）。 */
function findEndOfCentralDirectory(view: DataView): number {
  const minOffset = Math.max(0, view.byteLength - 22 - 0xffff)
  for (let offset = view.byteLength - 22; offset >= minOffset; offset--) {
    if (view.getUint32(offset, true) === EOCD_SIGNATURE) {
      return offset
    }
  }
  return -1
}

async function inflateRaw(data: Uint8Array): Promise<Uint8Array> {
  if (typeof DecompressionStream === 'undefined') {
    throw new Error('当前浏览器不支持解压升级包（需要 DecompressionStream）')
  }
  const stream = new Blob([data as BlobPart])
    .stream()
    .pipeThrough(new DecompressionStream('deflate-raw'))
  return new Uint8Array(await new Response(stream).arrayBuffer())
}

/** 读取 zip 内所有文件条目（目录项会被忽略）。 */
export async function readZipEntries(blob: Blob): Promise<ZipEntry[]> {
  const buffer = new Uint8Array(await blob.arrayBuffer())
  const view = new DataView(buffer.buffer)
  const decoder = new TextDecoder()

  const eocd = findEndOfCentralDirectory(view)
  if (eocd < 0) {
    throw new Error('不是有效的 zip 文件')
  }

  const entryCount = view.getUint16(eocd + 10, true)
  let offset = view.getUint32(eocd + 16, true)
  const entries: ZipEntry[] = []

  for (let i = 0; i < entryCount; i++) {
    if (offset + 46 > view.byteLength || view.getUint32(offset, true) !== CENTRAL_SIGNATURE) {
      throw new Error('zip 中央目录损坏')
    }

    const method = view.getUint16(offset + 10, true)
    const compressedSize = view.getUint32(offset + 20, true)
    const uncompressedSize = view.getUint32(offset + 24, true)
    const nameLength = view.getUint16(offset + 28, true)
    const extraLength = view.getUint16(offset + 30, true)
    const commentLength = view.getUint16(offset + 32, true)
    const localOffset = view.getUint32(offset + 42, true)
    const name = decoder.decode(buffer.subarray(offset + 46, offset + 46 + nameLength))
    offset += 46 + nameLength + extraLength + commentLength

    // 目录项以 / 结尾，没有内容
    if (name.endsWith('/')) continue

    if (view.getUint32(localOffset, true) !== LOCAL_SIGNATURE) {
      throw new Error(`zip 局部头损坏: ${name}`)
    }
    const localNameLength = view.getUint16(localOffset + 26, true)
    const localExtraLength = view.getUint16(localOffset + 28, true)
    const dataStart = localOffset + 30 + localNameLength + localExtraLength
    const raw = buffer.subarray(dataStart, dataStart + compressedSize)

    const read = async (): Promise<Uint8Array> => {
      if (method === 0) return raw.slice() // 存储（未压缩）
      if (method === 8) return inflateRaw(raw) // deflate
      throw new Error(`不支持的压缩方式(${method}): ${name}`)
    }

    entries.push({ name, size: uncompressedSize, read })
  }

  return entries
}
