using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class aligned_allocator<T, N> where T: notnull where N: notnull
    {
        // public class value_type : T { }
        public dynamic pointer = typeof(T);

        public static T address(T r) { return r; }

        public static aligned_allocator<T, N> operator --(aligned_allocator<T, N> value)
        {
            aligned_allocator<T, N> align = new aligned_allocator<T, N>();
            align.pointer--;
            return align;
        }

        public static aligned_allocator<T, N> operator ++(aligned_allocator<T, N> value)
        {
            aligned_allocator<T, N> align = new aligned_allocator<T, N>();
            align.pointer++;
            return align;
        }
    }

}
