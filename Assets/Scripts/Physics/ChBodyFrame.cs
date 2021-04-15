using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Class for objects that represent moving frames in space and
    /// that contain  ChVariables proxies to the solver.
    /// i.e. items with translational and rotational degrees of freedom
    /// This class is used as a base for the very used ChBody class
   // [System.Serializable]
    public class ChBodyFrame : ChFrameMoving<double>
    {
        public ChVariablesBodyOwnMass variables = new ChVariablesBodyOwnMass();  //< interface to solver (store inertia and coordinates)

        public ChBodyFrame() { }
        public ChBodyFrame(ChBodyFrame other) : base(other) { }


        /// Returns reference to the encapsulated ChVariablesBody,
        /// representing body variables (pos, speed or accel.- see VariablesLoad...() )
        /// and forces.
        /// The ChVariablesBodyOwnMass is the interface to the system solver.
        public virtual ChVariablesBodyOwnMass VariablesBody() { return variables; }
        public virtual ChVariables Variables() { return variables; }

        public void SetVariables(ChVariables var) { variables = (ChVariablesBodyOwnMass)var; }
        public void SetVariables(ChVariablesBodyOwnMass var) { variables = var; }

        /// Transform generic cartesian force into absolute force+torque applied to body COG.
        /// If local=1, force & application point are intended as expressed in local
        /// coordinates, if =0, in absolute.
        public void To_abs_forcetorque(ChVector force,
                                ChVector appl_point,
                                bool local,
                                ref ChVector resultforce,
                                ref ChVector resulttorque)
        {
            if (local)
            {
                // local space
                ChVector mforce_abs = TransformDirectionLocalToParent(force);
                resultforce = mforce_abs;
                resulttorque = ChVector.Vcross(TransformDirectionLocalToParent(appl_point), mforce_abs);
            }
            else
            {
                // absolute space
                resultforce = force;
                resulttorque = ChVector.Vcross(ChVector.Vsub(appl_point, coord.pos), force);
            }
        }

        /// Transform generic cartesian torque into absolute torque applied to body COG.
        /// If local=1, torque is intended as expressed in local coordinates, if =0, in absolute.
        public void To_abs_torque(ChVector torque, bool local, ref ChVector resulttorque) {
            if (local)
            {
                // local space
                resulttorque = this.TransformDirectionLocalToParent(torque);
            }
            else
            {
                // absolute space
                resulttorque = torque;
            }
        }

    }

}
