using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

	public static class ChGlobal
	{
		// -----------------------------------------------------------------------------
		// Functions for assigning unique identifiers
		// -----------------------------------------------------------------------------

		// Set the start value for the sequence of IDs (ATTENTION: not thread safe)
		// Subsequent calls to GetUniqueIntID() will return val+1, val+2, etc.
		static volatile int first_id = 100000;


		/// Obtain a unique identifier (thread-safe)
		/*public static int GetUniqueIntID()
		{
			int id = first_id;
			//return __sync_add_and_fetch(id, 1);
			//return ComputeBufferType.Counter()
		}*/

	}
}
