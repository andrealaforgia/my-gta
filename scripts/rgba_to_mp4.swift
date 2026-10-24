// Reads raw RGBA frames (bottom row first, as OpenGL reads them) from standard
// input and writes an H.264 MP4. Standard library and Apple frameworks only.
//
//   swiftc -O scripts/rgba_to_mp4.swift -o rgba_to_mp4
//   ./render_movie assets_cooked/cities/pessano_con_bornago | \
//       ./rgba_to_mp4 movie.mp4 1280 720 30
import AVFoundation
import CoreVideo
import Foundation

let arguments = CommandLine.arguments
guard arguments.count >= 5, let width = Int(arguments[2]),
  let height = Int(arguments[3]), let fps = Int32(arguments[4])
else {
  FileHandle.standardError.write("usage: rgba_to_mp4 out.mp4 width height fps\n".data(using: .utf8)!)
  exit(2)
}
let url = URL(fileURLWithPath: arguments[1])
try? FileManager.default.removeItem(at: url)
let writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
let settings: [String: Any] = [
  AVVideoCodecKey: AVVideoCodecType.h264,
  AVVideoWidthKey: width,
  AVVideoHeightKey: height,
  AVVideoCompressionPropertiesKey: [AVVideoAverageBitRateKey: 12_000_000],
]
let input = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
let adaptor = AVAssetWriterInputPixelBufferAdaptor(
  assetWriterInput: input,
  sourcePixelBufferAttributes: [
    kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
    kCVPixelBufferWidthKey as String: width,
    kCVPixelBufferHeightKey as String: height,
  ])
writer.add(input)
writer.startWriting()
writer.startSession(atSourceTime: .zero)

let frameBytes = width * height * 4
var raw = [UInt8](repeating: 0, count: frameBytes)
var frame: Int64 = 0
let stdin = FileHandle.standardInput

func readFrame() -> Bool {
  var have = 0
  while have < frameBytes {
    let chunk = stdin.readData(ofLength: frameBytes - have)
    if chunk.isEmpty { return false }
    chunk.copyBytes(to: &raw[have], count: chunk.count)
    have += chunk.count
  }
  return true
}

while readFrame() {
  while !input.isReadyForMoreMediaData { usleep(2000) }
  var buffer: CVPixelBuffer?
  CVPixelBufferPoolCreatePixelBuffer(nil, adaptor.pixelBufferPool!, &buffer)
  guard let pixels = buffer else { break }
  CVPixelBufferLockBaseAddress(pixels, [])
  let base = CVPixelBufferGetBaseAddress(pixels)!.assumingMemoryBound(to: UInt8.self)
  let stride = CVPixelBufferGetBytesPerRow(pixels)
  for y in 0..<height {
    // Flip, and turn RGBA into BGRA.
    let source = (height - 1 - y) * width * 4
    let target = y * stride
    for x in 0..<width {
      base[target + x * 4 + 0] = raw[source + x * 4 + 2]
      base[target + x * 4 + 1] = raw[source + x * 4 + 1]
      base[target + x * 4 + 2] = raw[source + x * 4 + 0]
      base[target + x * 4 + 3] = 255
    }
  }
  CVPixelBufferUnlockBaseAddress(pixels, [])
  adaptor.append(pixels, withPresentationTime: CMTime(value: frame, timescale: fps))
  frame += 1
}
input.markAsFinished()
let done = DispatchSemaphore(value: 0)
writer.finishWriting { done.signal() }
done.wait()
FileHandle.standardError.write("wrote \(frame) frames to \(arguments[1])\n".data(using: .utf8)!)
