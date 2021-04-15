using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{

    /// This is enough to use dynamic_casting<> to detect all template types
    /// from ChConstraintTwoTuplesFrictionT
    public class ChConstraintTwoTuplesFrictionTall {};

    /// Base class for friction constraints between two objects,
    /// each represented by a tuple of ChVariables objects.
    /// This constraint cannot be used alone. It must be used together with
    /// a ChConstraintTwoTuplesContactN
    public class ChConstraintTwoTuplesFrictionT<Ta, Tb> : ChConstraintTwoTuples<Ta, Tb> where Ta : notnull where Tb : notnull
    {
        private ChConstraintTwoTuplesFrictionTall m_ConstraintTwoTuplesFrictionTall = new ChConstraintTwoTuplesFrictionTall();


        /// Default constructor
        public ChConstraintTwoTuplesFrictionT() { this.mode = eChConstraintMode.CONSTRAINT_FRIC; }

        /// Copy constructor
        public ChConstraintTwoTuplesFrictionT(ChConstraintTwoTuplesFrictionT<Ta, Tb> other)
        : base(other) {}
}
}
