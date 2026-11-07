// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// Draws the device icon (a generic two-channel USB audio interface, front view)
// and writes an .iconset directory; `iconutil -c icns` turns it into .icns.
//
//   swift tools/makeicon.swift plugin/US122MKII.iconset && iconutil -c icns plugin/US122MKII.iconset

import CoreGraphics
import Foundation
import ImageIO
import UniformTypeIdentifiers

func draw(_ ctx: CGContext, _ size: CGFloat) {
    let s = size / 1024 // design on a 1024 grid
    ctx.scaleBy(x: s, y: s)
    let rgb = CGColorSpaceCreateDeviceRGB()
    func color(_ r: CGFloat, _ g: CGFloat, _ b: CGFloat, _ a: CGFloat = 1) -> CGColor {
        CGColor(colorSpace: rgb, components: [r, g, b, a])!
    }

    // body: a wide box seen slightly from above
    let body = CGRect(x: 72, y: 300, width: 880, height: 420)
    let bodyPath = CGPath(roundedRect: body, cornerWidth: 70, cornerHeight: 70, transform: nil)
    ctx.saveGState()
    ctx.setShadow(offset: CGSize(width: 0, height: -24), blur: 40, color: color(0, 0, 0, 0.35))
    ctx.addPath(bodyPath)
    ctx.setFillColor(color(0.16, 0.17, 0.19))
    ctx.fillPath()
    ctx.restoreGState()

    // brushed front panel with a vertical gradient
    let panel = body.insetBy(dx: 26, dy: 26)
    let panelPath = CGPath(roundedRect: panel, cornerWidth: 48, cornerHeight: 48, transform: nil)
    ctx.saveGState()
    ctx.addPath(panelPath)
    ctx.clip()
    let grad = CGGradient(colorsSpace: rgb, colors: [color(0.86, 0.87, 0.89), color(0.62, 0.64, 0.68)] as CFArray,
                          locations: [0, 1])!
    ctx.drawLinearGradient(grad, start: CGPoint(x: 0, y: panel.maxY), end: CGPoint(x: 0, y: panel.minY), options: [])
    ctx.restoreGState()

    // two combo input jacks (left), two gain knobs (middle), headphone jack (right)
    func jack(_ cx: CGFloat, _ cy: CGFloat, _ r: CGFloat) {
        ctx.setFillColor(color(0.12, 0.12, 0.14))
        ctx.fillEllipse(in: CGRect(x: cx - r, y: cy - r, width: 2 * r, height: 2 * r))
        ctx.setFillColor(color(0.32, 0.33, 0.36))
        let ri = r * 0.55
        ctx.fillEllipse(in: CGRect(x: cx - ri, y: cy - ri, width: 2 * ri, height: 2 * ri))
        ctx.setFillColor(color(0.05, 0.05, 0.06))
        let rh = r * 0.22
        ctx.fillEllipse(in: CGRect(x: cx - rh, y: cy - rh, width: 2 * rh, height: 2 * rh))
    }
    func knob(_ cx: CGFloat, _ cy: CGFloat, _ r: CGFloat, angle: CGFloat) {
        ctx.saveGState()
        ctx.setShadow(offset: CGSize(width: 0, height: -8), blur: 14, color: color(0, 0, 0, 0.4))
        ctx.setFillColor(color(0.10, 0.10, 0.12))
        ctx.fillEllipse(in: CGRect(x: cx - r, y: cy - r, width: 2 * r, height: 2 * r))
        ctx.restoreGState()
        let kg = CGGradient(colorsSpace: rgb, colors: [color(0.30, 0.31, 0.34), color(0.12, 0.12, 0.14)] as CFArray,
                            locations: [0, 1])!
        ctx.saveGState()
        let ri = r * 0.86
        ctx.addEllipse(in: CGRect(x: cx - ri, y: cy - ri, width: 2 * ri, height: 2 * ri))
        ctx.clip()
        ctx.drawLinearGradient(kg, start: CGPoint(x: cx, y: cy + ri), end: CGPoint(x: cx, y: cy - ri), options: [])
        ctx.restoreGState()
        // pointer line
        ctx.setStrokeColor(color(0.95, 0.95, 0.95))
        ctx.setLineWidth(r * 0.12)
        ctx.setLineCap(.round)
        ctx.move(to: CGPoint(x: cx + cos(angle) * r * 0.25, y: cy + sin(angle) * r * 0.25))
        ctx.addLine(to: CGPoint(x: cx + cos(angle) * r * 0.75, y: cy + sin(angle) * r * 0.75))
        ctx.strokePath()
    }
    func led(_ cx: CGFloat, _ cy: CGFloat, on: Bool) {
        ctx.saveGState()
        if on { ctx.setShadow(offset: .zero, blur: 18, color: color(0.2, 1.0, 0.35, 0.9)) }
        ctx.setFillColor(on ? color(0.25, 0.95, 0.4) : color(0.25, 0.3, 0.27))
        ctx.fillEllipse(in: CGRect(x: cx - 13, y: cy - 13, width: 26, height: 26))
        ctx.restoreGState()
    }

    let cy = panel.midY - 10
    jack(200, cy, 82)
    jack(390, cy, 82)
    knob(580, cy, 70, angle: .pi * 0.75)
    knob(745, cy, 70, angle: .pi * 0.35)
    led(580, cy + 118, on: true)
    led(745, cy + 118, on: false)
    jack(872, cy, 34)
}

let out = URL(fileURLWithPath: CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "US122MKII.iconset")
try? FileManager.default.createDirectory(at: out, withIntermediateDirectories: true)
for base in [16, 32, 128, 256, 512] {
    for scale in [1, 2] {
        let px = base * scale
        let ctx = CGContext(data: nil, width: px, height: px, bitsPerComponent: 8, bytesPerRow: 0,
                            space: CGColorSpaceCreateDeviceRGB(),
                            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
        draw(ctx, CGFloat(px))
        let name = scale == 1 ? "icon_\(base)x\(base).png" : "icon_\(base)x\(base)@2x.png"
        let dest = CGImageDestinationCreateWithURL(out.appendingPathComponent(name) as CFURL,
                                                   UTType.png.identifier as CFString, 1, nil)!
        CGImageDestinationAddImage(dest, ctx.makeImage()!, nil)
        CGImageDestinationFinalize(dest)
    }
}
print("wrote \(out.path)")
