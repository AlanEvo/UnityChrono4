using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using chrono;

namespace chrono
{

    public static class IntInterface
	{
        // Needs to be structs to be non-nullable for Generics


        public interface IBase
        {
            //public static int Masta { get; }
            public int Masta { get; }

        }
        public class VariableTupleCarrier_1vars<N1> : IBase
        {
            // public static dynamic Masta = 6;
            public class type_constraint_tuple : ChConstraintTuple_1vars<VariableTupleCarrier_1vars<N1>> { }
            public int Masta { get { return 1; } }

            public class nvars1 : IBase
            {
                // public static dynamic Masta = typeof(N1); 
                public int Masta { get { return 2; } }
            }
        }

        public interface ChVariableTupleCarrier_1vars : IBase// where N1 : notnull
        {
            // public static dynamic Masta = 6;
            public class type_constraint_tuple : ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars> { }

            public class nvars1 : IBase
            {
                // public static dynamic Masta = typeof(N1); 
                public int Masta { get { return 6;  } }
            }
            public abstract ChVariables GetVariables1();
        }


        public interface ChVariableTupleCarrier_3vars<N1, N2, N3> : IBase
        {
            public class type_constraint_tuple : ChConstraintTuple_3vars<ChVariableTupleCarrier_3vars<N1, N2, N3>> { }
            public class nvars1 : IBase { public int Masta { get { return 1; } } }
            public class nvars2 : IBase { public int Masta { get { return 2; } } }
            public class nvars3 : IBase { public int Masta { get { return 3; } } }
            public abstract ChVariables GetVariables1();
            public abstract ChVariables GetVariables2();
            public abstract ChVariables GetVariables3();
        }

        public class One : IBase
        {
            // public static int Masta = 1;
            public int Masta { get { return 1; } } 
        }

        public class Three : IBase
        {
            // public static int Masta = 3;
            public int Masta { get { return 3; } }
            //public int Flaps = 10;
            //public dynamic Masta2 = 3;//{ get { return 3; } }
        }



        public class Four : IBase
        {
            // public static int Masta = 4;
            public int Masta { get { return 4; } }

        }

        public class Six : IBase
        {
            // public static int Masta = 6;
            public int Masta { get { return 6; } }

        }

        public class Seven : IBase
        {
            // public static int Masta = 7;
            public int Masta { get { return 7; } }
        }

        public class SixtyFour : IBase
        {
            // public static int Masta = 64;
            public int Masta { get { return 64; } }
        }

    }
}
