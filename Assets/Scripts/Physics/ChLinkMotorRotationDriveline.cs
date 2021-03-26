using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{


    /// This is an "interface" from 3D to a powertrain/powertrain that is modeled via
    /// 1D elements such as ChShaft, ChShaftsMotor, ChShaftsGearbox, ChShaftsClutch, etc. 
    ///
    ///  This is the most avanced type of "motor" because using many of those 1D
    /// elements one can build very complex drivelines, for example use
    /// this ChLinkMotorRotationDriveline to represent a drive+reducer,
    /// hence taking into account of the inertia of the motor shaft (as in
    /// many cases of robotic actuators, that has electric drives+reducers.)
    /// At the same time, using 1D elements avoids the unnecessary complication 
    /// of using complete 3D parts to make fast spindles, 3D gears etc. 
    ///
    ///  The 1D driveline is "interfaced" to the two connected threedimensional
    /// parts using two "inner" 1D shafts, each rotating as the connected 3D part;
    /// it is up to the user to build the driveline that connects those two shafts.
    ///
    ///  Most often the driveline is a graph starting at inner shaft 2 (consider 
    /// it to be the truss for holding the motor drive, also the support for reducers 
    /// if any) and ending at inner shaft 1 (consider it to be the output, i.e. the 
    /// slow-rotation spindle).
    ///  Note that it is up to the user to create a driveline where all torques are
    /// balanced action/reactions: in this case, 
    ///    GetMotorTorque() = GetInnerTorque1() = - GetInnerTorque2(). 
    /// This is not true for example, for an unbalanced driveline where one of the 
    /// two inner shafts is connected to some external ChShaft. 

    public class ChLinkMotorRotationDriveline : ChLinkMotorRotation
    {

        /*  std::shared_ptr<ChShaft> innershaft1;
          std::shared_ptr<ChShaft> innershaft2;
          std::shared_ptr<ChShaftsBody> innerconstraint1;
          std::shared_ptr<ChShaftsBody> innerconstraint2;

          public:
          ChLinkMotorRotationDriveline();
          ChLinkMotorRotationDriveline(const ChLinkMotorRotationDriveline& other);*/
    }


}
