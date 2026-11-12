// A robot on the shared core Swift binding.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import Clunkyard

@main final class Bot:Robot {
    var free=false
    func steer(_ towards:Double,_ current:Double) throws -> Double {try angles.clamp(angles.angleDiff(towards,current)/45,-1,1)}
    override func onStart() throws {free=try match.is3d && arena.gravity.z==0}
    override func onTurn() throws {
        let position=try me.position
        try drive.forces(brake:0.2)
        if let target=try sensors.nearest {
            try drive.set(throttle:target.range>350 ? 0.7 : target.range<200 ? -0.4 : 0.2,turn:steer(target.bearing,me.heading),pitchTurn:steer(target.elevation,me.pitch))
            try scanner.set(heading:target.bearing,arc:20,pitch:target.elevation)
            let power=try gun.powerForFlightTime(target.range,1),aim=try gun.lead(target,power)
            let heading=aim?.heading ?? target.bearing,pitch=aim?.pitch ?? target.elevation,now=try gun.aimNow
            try gun.aim(heading:heading,pitch:pitch)
            if try abs(angles.angleDiff(heading,now.heading))<3 && abs(pitch-now.pitch)<3 && gun.canFire(power,10) {try gun.fire(power:power)}
            try status.set(text:"TRACK",value:target.range)
        }else if let cue=try tracker.cue {
            let bearing=try angles.bearingTo(position,cue.position),distance=try angles.distance(position,cue.position)
            let elevation=free ? try angles.elevationTo(position,cue.position) : 0,error=min(cue.bearingError,30),tick=try clock.tick
            let across=Double(tick%5-2)/2*error,up=free ? Double((tick/5)%3-1)*error/2 : 0
            try drive.set(throttle:0.7,turn:steer(bearing,me.heading),pitchTurn:steer(elevation,me.pitch))
            try scanner.set(heading:bearing+across,arc:scanner.arcForReach(distance+100),pitch:elevation+up)
            try status.set(text:"CUE",value:distance)
        }else{
            try drive.set(throttle:0.5,turn:0.33,pitchTurn:steer(0,me.pitch))
            let pitch=free ? Double((try clock.tick/10)%3-1)*35 : 0
            try scanner.set(heading:angles.normalize(scanner.last.heading+36),arc:40,pitch:pitch)
            try status.set(text:"SEARCH")
        }
        if try battery.charge<battery.capacity/4 {try drive.set(throttle:(drive.getThrottle() ?? 0)*0.5)}
        if let storm=try nav.storm,try (storm.margin(position) ?? -1e9) > -60 {
            try drive.set(throttle:0.8,turn:angles.clamp(angles.angleDiff(angles.bearingTo(storm.center,position),me.heading)/30,-1,1))
        }
        if let zone=try nav.zone,try (zone.margin(position) ?? 1e9)<60 {
            let climb=zone.shape==Shape.sphere ? try angles.elevationTo(position,zone.center) : 0
            try drive.set(throttle:0.8,turn:angles.clamp(angles.angleDiff(angles.bearingTo(position,zone.center),me.heading)/30,-1,1),pitchTurn:angles.clamp((climb-me.pitch)/30,-1,1))
        }
    }
    static func main() throws {try Bot().run()}
}
