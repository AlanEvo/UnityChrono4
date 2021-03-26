using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// This is enough to use dynamic_casting<> to detect all template types
    /// from ChConstraintTwoTuplesRollingT
    public class ChConstraintTwoTuplesRollingTall { };

    /// Base class for friction constraints between two objects,
    /// each represented by a tuple of ChVariables objects.
    /// This constraint cannot be used alone. It must be used together with
    /// a ChConstraintTwoTuplesContactN

    public class ChConstraintTwoTuplesRollingT<Ta, Tb> : ChConstraintTwoTuples<Ta, Tb> where Ta : notnull where Tb : notnull
    {
        protected ChConstraintTwoTuplesRollingTall chConstraintTwoTuplesRollingTall;

        /// Default constructor
        public ChConstraintTwoTuplesRollingT() { this.mode = eChConstraintMode.CONSTRAINT_FRIC; }

        /// Copy constructor
        public ChConstraintTwoTuplesRollingT(ChConstraintTwoTuplesRollingT<Ta, Tb> other) : base(other) { }


        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone() { return new ChConstraintTwoTuplesRollingT<Ta, Tb>(this); }

        /// Assignment operator: copy from other object
        /*  public ref ChConstraintTwoTuplesRollingT operator=(ChConstraintTwoTuplesRollingT other)
  {
      if (&other == this)
          return *this;

      // copy parent class data
      ChConstraintTwoTuples < Ta, Tb >::operator= (other);

      return *this;
  }*/

        /// Indicate whether or not this constraint is linear.
        public override bool IsLinear() { return false; }

        /// The constraint is satisfied?
        public override double Violation(double mc_i) { return 0.0; }
    };

}  // end namespace chrono
