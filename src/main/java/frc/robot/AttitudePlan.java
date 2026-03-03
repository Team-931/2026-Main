package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

class AttitudePlan {
    AttitudePlan(Rotation2d stayhere) {
        accel = 0;
        cruiseVel = 0;
        cruiseBase = stayhere;
        start = end = new State(stayhere, 0);
        cruiseTime = decelTime = endTime = 0;
    }
    AttitudePlan(Rotation2d start, Rotation2d end, double maxspd, double maxaccel) {
        this(start, 0, end, 0, maxspd, maxaccel);
    }    
    AttitudePlan(Rotation2d start, double startv, Rotation2d end, double endv, double maxspd, double maxaccel) {
        maxaccel = Math.abs(maxaccel);
        maxspd = Math.abs(maxspd);
        double change = end.minus(start).getRadians();
        double a_change = maxaccel * change, 
            v_min = Math.min(startv, endv), 
            v_max = Math.max(startv, endv), 
            v_min2 = v_min*v_min/2, 
            v_max2 = v_max*v_max/2,
            discrim = a_change + v_min2 - v_max2, // TODO -- cases if (discrim == 0) or v_max > maxspd ...
            noCruise = Math.copySign(a_change, discrim) + v_min2 + v_max2;
        accel = Math.copySign(maxaccel, discrim);
        double helpCruise = noCruise - maxspd * maxspd,
         cruiseLen = 0;
        if (helpCruise <= 0) 
            cruiseVel = Math.copySign(Math.sqrt(noCruise), discrim);
        else {
            cruiseVel = Math.copySign(maxspd, discrim);
            cruiseLen = helpCruise / cruiseVel / accel;
        }
        cruiseTime = (cruiseVel - startv) / accel;
        decelTime = cruiseTime + cruiseLen;
        endTime = (cruiseVel - endv) / accel + decelTime;
        this.start = new State(start, startv);
        this.end = new State(end, endv);
        cruiseBase = start.minus(new Rotation2d(cruiseLen*cruiseVel/2));
    }    
    State report(double time) {
        if(time <= 0) return start;
        if(time <= cruiseTime)
            return new State(start.angle.plus(new Rotation2d(time * (start.rotSpeed + accel * time / 2))), start.rotSpeed + accel * time);
        if (time <= decelTime) return new State(cruiseBase.plus(new Rotation2d(time*cruiseVel)), cruiseVel);
        if(time < endTime) {
            time = endTime - time;
            return new State(end.angle.minus(new Rotation2d(time * (end.rotSpeed + accel * time / 2))), end.rotSpeed + accel * time);
        }
        return end;
    }
    /** Represents one stage of the plan */
    static class State {
        /** direction to face (possibly relative to some external landmark) */
        final Rotation2d angle;
        /** change in {@code angle} in radians/ sec */
        final double rotSpeed;
        /** @param currentAngle sets the {@code angle} member
         * @param radiansPerSec sets the {@code rotSpeed} member
         */
        State (Rotation2d currentAngle,  double radiansPerSec) {
            angle = currentAngle;
            rotSpeed = radiansPerSec;
        }
        static State kZero = new State(Rotation2d.kZero, 0);
    }

    //TODO: members 
   final double accel, cruiseVel;
   final Rotation2d cruiseBase;
   final State start, end;
   final double cruiseTime, decelTime, endTime;
}
