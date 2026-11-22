import Clunkyard
import Foundation

private let searchArc = 40.0  // degrees: sweeps the circle in half a second
private let holdWidth = 60.0  // units across at the target: it stays in the beam from tick to tick
private let minArc = 6.0  // degrees: the narrowest the beam holds
private let window: Int32 = 12  // ticks of sightings the aim averages
private let sightings = 8  // narrow sightings in the window before a shot
private let lookEvery: Int32 = 40  // ticks between looks around the target, for a nearer robot
private let reacquire: Int32 = 10  // ticks to look again where a lost target was

@main
final class Bot: Robot {
    private var free = false
    private var dodgeTicks = 0
    private var dodges = 0
    private var slot: Int32 = 0

    private func steer(_ towards: Double, _ current: Double) throws -> Double {
        try angles.clamp(angles.angleDiff(towards, current) / 45, -1, 1)
    }
    // A straight line through the narrowest of the last `window` ticks' sightings:
    // where the robot is now and how fast it moves. Each sighting is off by its
    // bearingError; the line averages that out.
    private func fit(_ slot: Int32, _ tick: Int32) throws -> (
        position: Vec3, velocity: Vec3, count: Int
    )? {
        let recent = try tracker.history(slot).filter { $0.tick > tick - window }
        guard recent.count >= 2, let best = recent.map({ $0.bearingError }).min() else {
            return nil
        }
        let seen = recent.filter { $0.bearingError <= 1.5 * best }
        let n = Double(seen.count)
        guard seen.count >= 2 else { return nil }
        let t0 = seen.reduce(0.0) { $0 + Double($1.tick) } / n
        let spread = seen.reduce(0.0) { $0 + pow(Double($1.tick) - t0, 2) }
        let rate = Double(try match.tickRate)
        // Least squares on one axis: its fitted position now and its slope per tick.
        func line(_ axis: KeyPath<Vec3, Double>) -> (now: Double, slope: Double) {
            let mean = seen.reduce(0.0) { $0 + $1.position[keyPath: axis] } / n
            let slope =
                seen.reduce(0.0) {
                    $0 + (Double($1.tick) - t0) * ($1.position[keyPath: axis] - mean)
                } / spread
            return (mean + slope * (Double(tick) - t0), slope)
        }
        let x = line(\.x)
        let y = line(\.y)
        let z = line(\.z)
        return (
            Vec3(x: x.now, y: y.now, z: z.now),
            Vec3(x: x.slope * rate, y: y.slope * rate, z: z.slope * rate), seen.count
        )
    }

    override func onStart() throws {
        free = try match.is3d && arena.gravity.z == 0
    }

    override func onTurn() throws {
        let position = try me.position
        let tick = try clock.tick
        let last = try scanner.last
        let lastSeen = slot != 0 ? try tracker.lastSeen(slot) : nil
        try drive.forces(brake: 0.2)
        if let target = try sensors.nearest {
            slot = target.slot
            try drive.set(
                throttle: target.range > 350 ? 0.7 : target.range < 200 ? -0.4 : 0.2,
                turn: steer(target.bearing, me.heading),
                pitchTurn: steer(target.elevation, me.pitch))
            // Halve the beam each tick until it is holdWidth across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            let hold = max(minArc, 2 * atan2(holdWidth / 2, target.range) * 180 / .pi)
            // In a crowd a narrow beam sees one robot. Every lookEvery ticks, widen it for a tick.
            let arc = tick % lookEvery == 0 ? searchArc : max(hold, min(searchArc, last.arc / 2))
            try scanner.set(heading: target.bearing, arc: arc, pitch: target.elevation)
            let power = try gun.powerForFlightTime(target.range, 1)
            // An exact contact: gun.lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            var aim: Solution?
            var ready = true
            if target.bearingError > 0 {
                let line = try fit(target.slot, tick)
                aim = try line.flatMap { try gun.leadPoint($0.position, $0.velocity, power) }
                ready = (line?.count ?? 0) >= sightings && aim != nil
            } else {
                aim = try gun.lead(target, power)
            }
            let heading = aim?.heading ?? target.bearing
            let pitch = aim?.pitch ?? target.elevation
            let now = try gun.aimNow
            try gun.aim(heading: heading, pitch: pitch)
            if try ready && abs(angles.angleDiff(heading, now.heading)) < 3
                && abs(pitch - now.pitch) < 3 && gun.canFire(power, 10)
            {
                try gun.fire(power: power)
            }
            try status.set(text: ready ? "TRACK" : "NARROW", value: target.range)
        } else if let seen = lastSeen, tick - seen <= reacquire {
            // Lost it: look again where it was last seen, widening the beam, before searching.
            let spot = try tracker.position(slot) ?? position
            let bearing = try angles.bearingTo(position, spot)
            let elevation = free ? try angles.elevationTo(position, spot) : 0
            try drive.set(
                throttle: 0.5, turn: steer(bearing, me.heading),
                pitchTurn: steer(elevation, me.pitch))
            try scanner.set(heading: bearing, arc: min(searchArc, last.arc * 2), pitch: elevation)
            try status.set(text: "REACQUIRE")
        } else if let cue = try tracker.cue {
            let bearing = try angles.bearingTo(position, cue.position)
            let distance = try angles.distance(position, cue.position)
            let elevation = free ? try angles.elevationTo(position, cue.position) : 0
            let error = min(cue.bearingError, 30)
            let across = Double(tick % 5 - 2) / 2 * error
            let up = free ? Double((tick / 5) % 3 - 1) * error / 2 : 0
            try drive.set(
                throttle: 0.7, turn: steer(bearing, me.heading),
                pitchTurn: steer(elevation, me.pitch))
            try scanner.set(
                heading: bearing + across, arc: scanner.arcForReach(distance + 100),
                pitch: elevation + up)
            try status.set(text: "CUE", value: distance)
        } else {
            try drive.set(throttle: 0.5, turn: 0.33, pitchTurn: steer(0, me.pitch))
            let pitch = free ? Double((tick / 10) % 3 - 1) * 35 : 0
            try scanner.set(
                heading: angles.normalize(scanner.last.heading + 36), arc: searchArc, pitch: pitch)
            try status.set(text: "SEARCH")
        }

        if try dodgeTicks == 0 && sensors.cues.contains(where: { $0.source == Source.muzzle }) {
            dodges += 1
            dodgeTicks = 10
        }
        if dodgeTicks > 0 {
            // Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
            let left = dodges % 2 == 1
            let side: Double = left ? 1 : -1
            dodgeTicks -= 1
            if try match.control == Control.forces {
                try drive.set(throttle: 0.5)
                try drive.forces(strafe: -side)
            } else {
                try drive.set(throttle: 1, turn: side)
            }
            try status.set(text: left ? "dodge:left" : "dodge:right", value: Double(dodges))
        }
        if try battery.charge < battery.capacity / 4 {
            try drive.set(throttle: (drive.getThrottle() ?? 0) * 0.5)
        }
        if let storm = try nav.storm, try (storm.margin(position) ?? -1e9) > -60 {
            try drive.set(
                throttle: 0.8,
                turn: angles.clamp(
                    angles.angleDiff(angles.bearingTo(storm.center, position), me.heading) / 30, -1,
                    1))
        }
        if let zone = try nav.zone, try (zone.margin(position) ?? 1e9) < 60 {
            let climb =
                zone.shape == Shape.sphere ? try angles.elevationTo(position, zone.center) : 0
            try drive.set(
                throttle: 0.8,
                turn: angles.clamp(
                    angles.angleDiff(angles.bearingTo(position, zone.center), me.heading) / 30, -1,
                    1), pitchTurn: angles.clamp((climb - me.pitch) / 30, -1, 1))
        }
    }

    static func main() throws {
        try Bot().run()
    }
}
