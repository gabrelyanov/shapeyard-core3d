import AppKit
import MetalKit

/// Compile and initialize the actual mobile renderer against the macOS SDK.
/// This qualifies shader/pipeline availability, not visible asset presentation.
@main
struct SharedMetalRendererProbe {
    @MainActor
    static func main() throws {
        _ = NSApplication.shared
        guard let device = MTLCreateSystemDefaultDevice() else {
            throw NSError(domain: "ShapeyardMacMetalProbe", code: 1,
                          userInfo: [NSLocalizedDescriptionKey: "No Metal device"])
        }
        let view = MTKView(frame: NSRect(x: 0, y: 0, width: 640, height: 480),
                           device: device)
        view.isPaused = true
        view.enableSetNeedsDisplay = true
        view.colorPixelFormat = .bgra8Unorm_srgb
        view.depthStencilPixelFormat = .depth32Float
        let renderer = try ModelEditorMetalRenderer(view: view)
        view.delegate = renderer
        // Exercise AppKit invalidation; no fabricated scene is published.
        renderer.requestAnotherPresentation()
        RunLoop.main.run(until: Date(timeIntervalSinceNow: 0.03))
        renderer.cancelPickRequests()
        view.delegate = nil
        renderer.view = nil
        print("PASS: actual shared Metal renderer initializes shaders/pipelines on macOS; asset presentation pending")
    }
}
