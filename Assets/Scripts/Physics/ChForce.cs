using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Forces are objects which must be attached to rigid bodies in order
    /// to apply torque or force to such body. ChForce objects are able to
    /// represent either forces and torques, depending on a flag.
    public class ChForce : ChObj
    {

        // Types of force application
        public enum ForceType { FORCE, TORQUE };

        // Reference for position frame
        public enum ReferenceFrame { BODY, WORLD };

        // Reference for alignment
        public enum AlignmentFrame { BODY_DIR, WORLD_DIR };


        private ChBody Body;  //< object of application

        private ForceType mode;        //< force or torque
        private ReferenceFrame frame;  //< fix position in body csys or world csys
        private AlignmentFrame align;  //< fix direction in body csys or world csys

        private ChVector vpoint = new ChVector(0, 0, 0);     //< absolute point of application
        private ChVector vrelpoint = new ChVector(0, 0, 0);  //< relative point of application

        private ChFunction move_x;  /// motion x (abs or rel, depends on 'frame')
        private ChFunction move_y;  //< motion y  ""
        private ChFunction move_z;  //< motion z  ""
        private ChVector restpos = new ChVector(0, 0, 0);                  //< t=0 position (abs or rel, depends on 'frame')

        private ChFunction f_x;  //< fv strengh x (abs or rel, see 'align')
        private ChFunction f_y;  //< fv strengh y  ""
        private ChFunction f_z;  //< fv strengh z  ""

        private double mforce;                       //< fm scalar force strenght
        private ChFunction modula;  //< scalar force fm modulation

        private ChVector vdir = new ChVector(0, 0, 0);     ///< force/torque abs.direction
        private ChVector vreldir = new ChVector(0, 0, 0);  ///< force/torque rel direction

        private ChVector force = new ChVector(0, 0, 0);     //< TOTAL force vect (abs.coord) = fm*vdir +fx+fy+fz
        private ChVector relforce = new ChVector(0, 0, 0);  //< TOTAL force vect (rel.coord) = fm*vdir +fx+fy+fz

        private ChMatrixDynamic<double> Qf = new ChMatrixDynamic<double>();  //< Lagrangian force


        public ChForce()
        {
            Body = null;
            vpoint = new ChVector(0, 0, 0);
            vrelpoint = new ChVector(0, 0, 0);
            force = new ChVector(0, 0, 0);
            relforce = new ChVector(0, 0, 0);
            vdir = ChVector.VECT_X;
            vreldir = ChVector.VECT_X;
            restpos = new ChVector(0, 0, 0);
            mforce = 0;
            align = AlignmentFrame.BODY_DIR;
            frame = ReferenceFrame.BODY;
            mode = ForceType.FORCE;
            Qf = new ChMatrixDynamic<double>(7, 1);

            modula = new ChFunction_Const(1);
            move_x = new ChFunction_Const(0);
            move_y = new ChFunction_Const(0);
            move_z = new ChFunction_Const(0);
            f_x = new ChFunction_Const(0);
            f_y = new ChFunction_Const(0);
            f_z = new ChFunction_Const(0);
        }


        public ChForce(ChForce other) { }


        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChForce(this);
    }

        /// Return the parent body (the force belongs to this rigid body)
        public ChBody GetBody() { return Body; }
        /// Sets the parent body (the force belongs to this rigid body)
        public void SetBody(ChBody newRB) { Body = newRB; }

        /// Sets the mode (force or torque)
        public void SetMode(ForceType m_mode) { mode = m_mode; }
        public ForceType GetMode() { return mode; }

        /// Sets the alignment method.
        /// The force will rotate together with this reference.
        public void SetAlign(AlignmentFrame m_align) { align = m_align; }
        public AlignmentFrame GetAlign() { return align; }

        /// Sets the alignment method.
        /// The force application point will follow this reference.
        public void SetFrame(ReferenceFrame m_frame)
        {
            frame = m_frame;
            SetVpoint(vpoint);
        }
        public ReferenceFrame GetFrame() { return frame; }

        /// Gets the application point, in absolute coordinates.
        public ChVector GetVpoint() { return vpoint; }
        /// Gets the application point, in rigid body coordinates.
        public ChVector GetVrelpoint() { return vrelpoint; }

        /// Gets the application point, in absolute coordinates.
        public void SetVpoint(ChVector mypoint) { }
        /// Gets the application point, in rigid body coordinates.
        public void SetVrelpoint(ChVector myrelpoint) { }

        /// Gets the force (or torque) direction, in absolute coordinates.
        public ChVector GetDir() { return vdir; }
        /// Gets the force (or torque) direction, in rigid body coordinates.
        public ChVector GetRelDir() { return vreldir; }
        /// Sets the force (or torque) direction, in absolute coordinates.
        public void SetDir(ChVector newf) { }
        /// Sets the force (or torque) direction, in rigid body coordinates.
        public void SetRelDir(ChVector newf) { }

        /// Sets force (or torque) modulus.
        public void SetMforce(double newf) { }
        /// Gets force (or torque) modulus.
        public double GetMforce() { return mforce; }

        /// Sets a f(t) function for time-modulation of the force.
        public void SetModulation(ChFunction m_funct) { modula = m_funct; }
        public ChFunction GetModulation() { return modula; }

        /// Sets a f(t) function for time dependency of position (on x axis)
        public void SetMove_x(ChFunction m_funct) { move_x = m_funct; }
        public ChFunction GetMove_x() { return move_x; }
        /// Sets a f(t) function for time dependency of position (on y axis)
        public void SetMove_y(ChFunction m_funct) { move_y = m_funct; }
        public ChFunction GetMove_y() { return move_y; }
        /// Sets a f(t) function for time dependency of position (on z axis)
        public void SetMove_z(ChFunction m_funct) { move_z = m_funct; }
        public ChFunction GetMove_z() { return move_z; }

        /// Sets a f(t) function for time dependency of force X component.
        public void SetF_x(ChFunction m_funct) { f_x = m_funct; }
        public ChFunction GetF_x() { return f_x; }
        /// Sets a f(t) function for time dependency of force Y component.
        public void SetF_y(ChFunction m_funct) { f_y = m_funct; }
        public ChFunction GetF_y() { return f_y; }
        /// Sets a f(t) function for time dependency of force Z component.
        public void SetF_z(ChFunction m_funct) { f_z = m_funct; }
        public ChFunction GetF_z() { return f_z; }

        /// Gets the instant force vector -or torque vector- in absolute coordinates.
        public ChVector GetForce() { return force; }
        /// Gets the instant force vector -or torque vector- in rigid body coordinates.
        public ChVector GetRelForce() { return relforce; }
        /// Gets the instant force vector -or torque vector- modulus.
        public double GetForceMod() { return force.Length(); }

        /// Gets force-torque applied to rigid body, as lagrangian generalized force (7x1 matrix).
        public ChMatrix GetQf() { return Qf; }
        /// Gets force-torque applied to rigid body, as force vector (in absol.coords)
        /// and torque vector (in body coords).
        public void GetBodyForceTorque(ref ChVector body_force, ref ChVector body_torque) { }

        //
        // UPDATING
        //

        public void UpdateTime(double mytime) { }
        public void UpdateState() { }
        public void update(double mytime) { 
        
        }


    }
}
