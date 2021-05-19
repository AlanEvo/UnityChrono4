using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;


namespace chrono
{

    /** \class ChSparsityPatternLearner
\brief A dummy matrix that acquires the sparsity pattern.

ChSparsityPatternLearner estimates the sparsity pattern without actually allocating any value, but the elements indexes.
Other matrices (like ChCSMatrix) can acquire the sparsity pattern information from this matrix.
*/
    public class ChSparsityPatternLearner : ChSparseMatrix, System.IDisposable
    {

        protected List<List<int>> leadDim_list = new List<List<int>>();
        protected bool row_major_format = true;
        protected int leading_dimension;
        protected int trailing_dimension;


        public ChSparsityPatternLearner(int nrows, int ncols, bool row_major_format_in = true) : base(nrows, ncols)
        {
            row_major_format = row_major_format_in;
            // leading_dimension = (int[])Array.CreateInstance(typeof(Int32), row_major_format ? m_num_rows : m_num_cols);
            // trailing_dimension = (int[])Array.CreateInstance(typeof(Int32), row_major_format ? m_num_cols : m_num_rows);
            leading_dimension = row_major_format ? m_num_rows : m_num_cols;
            trailing_dimension = row_major_format ? m_num_cols : m_num_rows;
            leadDim_list.Capacity = leading_dimension;
        }
        public virtual void Dispose()
        {
        }

        public override void SetElement(int insrow, int inscol, double insval, bool overwrite = true)
        {
           // row_major_format ? leadDim_list[insrow].push_back(inscol) : leadDim_list[inscol].push_back(insrow);
        }

        public override double GetElement(int row, int col) { return 0.0; }

        public override void Reset(int row, int col, int nonzeros = 0)
        {
            //leading_dimension = (int[])Array.CreateInstance(typeof(Int32), row_major_format ? row : col);
            //trailing_dimension = (int[])Array.CreateInstance(typeof(Int32), row_major_format ? col : row);
            leading_dimension = row_major_format ? row : col;
            trailing_dimension = row_major_format ? col : row;
            leadDim_list.Clear();
            leadDim_list.Capacity = leading_dimension;
        }

        public override bool Resize(int nrows, int ncols, int nonzeros = 0)
        {
            Reset(nrows, ncols, nonzeros);
            return true;
        }

        public List<List<int>> GetSparsityPattern()
        {
            /*  for (auto list_iter = leadDim_list.begin(); list_iter != leadDim_list.end(); ++list_iter)
              {
                  list_iter->sort();
                  list_iter->unique();
              }*/
            return leadDim_list;
        }

        public bool IsRowMajor() { return row_major_format; }

        public override int GetNNZ()
        {
            int nnz_temp = 0;
            foreach (var list_iter in leadDim_list)
                nnz_temp += (int)(list_iter.Count);

            // (ChSparsityPatternLearner)(this).m_nnz = nnz_temp;
            return nnz_temp;
        }
    }


    public unsafe class ChCSMatrix : ChSparseMatrix
    {

        private bool row_major_format = true;  //< if \c true the arrays are stored according to CSR format, otherwise in CSC format
                                               // public static int array_alignment = 64;  //< alignment of arrays [byte]
        private bool isCompressed = false;  //< if \c true the arrays are compressed, so there is no not-initialized element
        private int max_shifts = int.MaxValue;

        public ChCSMatrix flaps;

        // Compressed Storage arrays typedefs   

        public class index_vector_t : Dictionary<int, aligned_allocator<int, IntInterface.SixtyFour>> { }  //< type for #leadIndex and #trailIndex vectors (aligned)
        public class values_vector_t : Dictionary<double, aligned_allocator<double, IntInterface.SixtyFour>> { }

        public index_vector_t leadIndex = new index_vector_t();   //< CS vector: #leadIndex[i] tells that #trailIndex[#leadIndex[i]] is the first element
        ///of the i-th row (if row-major)
        public index_vector_t trailIndex = new index_vector_t();  //< CS vector: #trailIndex[j] tells the column index of #values[j] (if row-major)
        public values_vector_t values = new values_vector_t();     //< CS vector: non-zero values
        public List<bool> initialized_element = new List<bool>();  //< flag if a space in #trailIndex is initialized or not
        public int leading_dimension;       //< points to #m_num_rows (CSR) or #m_num_cols (CSC)
        public int trailing_dimension;      //< points to #m_num_cols (CSR) or #m_num_rows (CSC)

        bool m_lock_broken = false;  //< true if a modification was made that overrules m_lock

        /// Create a CS matrix with the given dimensions.
        public ChCSMatrix(int nrows = 1,                    //< number of rows
               int ncols = 1,                    //< number of columns
               bool row_major_format_on = true,  //< create a Row Major matrix?
               int nonzeros = 1                  //< number of non-zeros
    ) : base(nrows, ncols)
        {
            row_major_format = row_major_format_on;
            // leading_dimension = new int[row_major_format ? m_num_rows : m_num_cols];
            // trailing_dimension = new int[row_major_format ? m_num_cols : m_num_rows];
            leading_dimension = row_major_format ? m_num_rows : m_num_cols;
            trailing_dimension = row_major_format ? m_num_cols : m_num_rows;

            reset_arrays(leading_dimension, trailing_dimension, nonzeros);
        }

        /// (internal) The \a vector elements will contain equally spaced indexes, going from \a initial_number to \a
        /// final_number.
        public static void distribute_integer_range_on_vector(index_vector_t vector, int initial_number, int final_number)
        {
            double delta = (double)(final_number - initial_number) / (vector.Count - 1);
            for (int el_sel = 0; el_sel < vector.Count; el_sel++)
            {
                vector.ElementAt((int)(Math.Round(delta * el_sel)));
            }
        }

        /// Get the length of the trailing-index array (e.g. column index if row major, row index if column major)
        public int GetTrailingIndexLength() { return leadIndex.Count; }

        /// (internal) Really reset the internal arrays to the given dimensions.
        /// It checks if the dimensions are consistent, but no smart choice are made here.
        protected void reset_arrays(int lead_dim, int trail_dim, int nonzeros)
        {
            // break sparsity lock
            m_lock_broken = true;

            // update dimensions (redundant if called from constructor)
            leading_dimension = lead_dim;
            trailing_dimension = trail_dim;

            // check if there is at least one element per row
            // nonzeros = Math.Max(nonzeros, lead_dim);

            // allocate the arrays
            // leadIndex.Capacity = leading_dimension[0] + 1;
           // ListExtras.Resize<int, aligned_allocator<int, IntInterface.SixtyFour>>(leadIndex, leading_dimension + 1);
           // ListExtras.Resize<int, aligned_allocator<int, IntInterface.SixtyFour>>(trailIndex, nonzeros);
           // ListExtras.Resize<double, aligned_allocator<double, IntInterface.SixtyFour>>(values, nonzeros);
           // ListExtras.Resize<bool>(initialized_element, nonzeros);
            /* for(int i = 0; i <= initialized_element.Count; i++)
             {
                 initialized_element.Add(false);
             }*/


            // make leadIndex span over available space
            distribute_integer_range_on_vector(leadIndex, 0, nonzeros);

            //// set the number of max shifts
            // max_shifts = GetNNZ() * 2 / *leading_dimension;

            isCompressed = false;
        }

        /// (internal) Insert a non existing element in the position \a trai_i, given the row(CSR) or column(CSC) \a
        /// lead_sel
        protected void insert(ref int trail_i_sel, int lead_sel)
        {
            isCompressed = false;
            m_lock_broken = true;

            bool OK_also_out_of_row = true;  // look for viable positions also in other rows respect to the one selected
            bool OK_also_onelement_rows = false;

            int trailIndexlength = leadIndex.Count;
            int shift_fw = 0;  // 0 means no viable position found forward
            int shift_bw = 0;  // 0 means no viable position found backward

            // TODO: optimize?
            // look for not-initialized elements FORWARD
            var lead_sel_fw = lead_sel;
            for (var trail_i = trail_i_sel + 1; trail_i < trailIndexlength && (trail_i - trail_i_sel) < max_shifts;
                 ++trail_i)
            {
               /* if (!initialized_element[trail_i])  // look for not initialized elements
                {
                    // check if it is out of row
                   // if (!OK_also_out_of_row && trail_i >= leadIndex[lead_sel + 1])
                    //    break;

                    // check if it is in 1element row
                    if (!OK_also_onelement_rows)  // check if we are out of the starting row
                    {
                        // find the beginning of row that follows the one in which we have found an initialized space
                        for (; leadIndex[lead_sel_fw] <= trail_i; ++lead_sel_fw)
                        {
                        }
                        if (leadIndex[lead_sel_fw - 1] + 1 >= leadIndex[lead_sel_fw])
                            continue;
                    }
                    shift_fw = trail_i - trail_i_sel;
                    break;
                }*/
            }

            // look for not-initialized elements BACWARD
            var lead_sel_bw = lead_sel;
            if (OK_also_out_of_row)  // do it only if 'out of row' insertions are allowed
            {
                for (var trail_i = trail_i_sel - 1;
                     trail_i >= 0 && (trail_i_sel - trail_i) < Math.Min(max_shifts, Math.Max(shift_fw, 1)); --trail_i)
                {
                    if (!initialized_element[trail_i])  // look for not initialized elements
                    {
                        // check if it is in 1element row
                        if (!OK_also_onelement_rows)  // check if we are out of the starting row
                        {
                            // find the beginning of row that follows the one in which we have found an initialized space
                           /* for (; leadIndex[lead_sel_bw] > trail_i; --lead_sel_bw)
                            {
                            }
                            if (leadIndex[lead_sel_bw + 1] - 1 <= leadIndex[lead_sel_bw])
                                continue;*/
                        }
                        shift_bw = trail_i - trail_i_sel;
                        break;
                    }
                }
            }

            if (shift_bw == 0 && shift_fw == 0)
            {
                // no viable position found
                // some space has to be made right where trail_i_sel is pointing
                // meanwhile we create some additional space also to all the other rows
                // in order to reduce the risk of another redistribution
                // so trail_i_sel WILL CHANGE

                var desired_storage_augmentation = (int)(Math.Max(GetTrailingIndexLength() * 0.2, 1.0));
                // trail_i_sel = Inflate(desired_storage_augmentation, lead_sel, trail_i_sel);

            }
            else
            {
                // shift the elements in order to have a not initialized position where trail_i_sel points
                // trail_i_sel WILL CHANGE if backward, WON'T CHANGE if forward
                // WARNING! move_backward is actually forward (towards the end of the array)
                if (shift_bw < 0 && -shift_bw < shift_fw)
                {
                    if (shift_bw < -1)
                    {
                        // move is not actually 'moving'; it's just for cleaner code!
                        //std::move(trailIndex.begin() + trail_i_sel + shift_bw + 1, trailIndex.begin() + trail_i_sel,
                        //trailIndex.begin() + trail_i_sel + shift_bw);
                       /* ShiftElement(trailIndex, trailIndex.First().Key + trail_i_sel + shift_bw + 1,
                                  trailIndex.First().Key + trail_i_sel + shift_bw);*/
                        //std::move(values.begin() + trail_i_sel + shift_bw + 1, values.begin() + trail_i_sel,
                        //values.begin() + trail_i_sel + shift_bw);
                       /* ShiftElement(values, values.First().Key + trail_i_sel + shift_bw + 1,
                                 values.First().Key + trail_i_sel + shift_bw);*/
                        // std::move(initialized_element.begin() + trail_i_sel + shift_bw + 1,
                        //  initialized_element.begin() + trail_i_sel,
                        //  initialized_element.begin() + trail_i_sel + shift_bw);
                        ShiftElement(initialized_element,
                                  initialized_element.First() + (dynamic)trail_i_sel + shift_bw + 1,
                                  initialized_element.First() + (dynamic)trail_i_sel + shift_bw);
                    }

                  /*  for (lead_sel_bw = lead_sel; leadIndex[lead_sel_bw] > trail_i_sel + shift_bw; --lead_sel_bw)
                    {
                        int i = 0;
                        leadIndex[lead_sel_bw]--;

                        var array = new int[5];
                        array[0] = 1;
                        array[0]++;

                        leadIndex[lead_sel_fw]--;
                        //Tuple.Create(leadIndex[lead_sel_fw].Item1 - 1);
                        // leadIndex.Resize(-1);
                    }*/

                    trail_i_sel--;

                }
                else
                {
                    // move is not actually 'moving'; it's just for cleaner code!
                    // std::move_backward(trailIndex.begin() + trail_i_sel, trailIndex.begin() + trail_i_sel + shift_fw,
                    //                   trailIndex.begin() + trail_i_sel + shift_fw + 1);
                    //ShiftElement(trailIndex, trailIndex.First() + trail_i_sel,
                     //                  trailIndex.First() + trail_i_sel + shift_fw + 1);
                    // std::move_backward(values.begin() + trail_i_sel, values.begin() + trail_i_sel + shift_fw,
                    //            values.begin() + trail_i_sel + shift_fw + 1);
                   // ShiftElement(values, values.First() + trail_i_sel,
                      //                 values.First() + trail_i_sel + shift_fw + 1);
                    //std::move_backward(initialized_element.begin() + trail_i_sel,
                    //         initialized_element.begin() + trail_i_sel + shift_fw,
                    //       initialized_element.begin() + trail_i_sel + shift_fw + 1);
                    ShiftElement(initialized_element,
                                       initialized_element.First() + (dynamic)trail_i_sel,
                                       initialized_element.First() + (dynamic)trail_i_sel + shift_fw + 1);

                   /*for (lead_sel_fw = lead_sel + 1; leadIndex[lead_sel_fw] <= trail_i_sel + shift_fw; ++lead_sel_fw)
                    {
                        leadIndex[lead_sel_fw]++;                   
                    }*/
                }
            }
            // let the returning function store the value, takes care of initialize element and so on...
        }

        public void IncrementCount(Dictionary<int, int> someDictionary, int id)
        {
            if (!someDictionary.ContainsKey(id))
                someDictionary[id] = 0;

            someDictionary[id]++;
        }

        public void ShiftElement<T>(List<Tuple<T, aligned_allocator<T, IntInterface.SixtyFour>>> array, T oldIndex, T newIndex) where T : struct
        {
            // TODO: Argument validation
            if (oldIndex == (dynamic)newIndex)
            {
                return; // No-op
            }
            T tmp = array[(dynamic)oldIndex];
            if (newIndex < (dynamic)oldIndex)
            {
                // Need to move part of the array "up" to make room
                Array.Copy(array.ToArray(), (dynamic)newIndex, array.ToArray(), newIndex + (dynamic)1, oldIndex - (dynamic)newIndex);
            }
            else
            {
                // Need to move part of the array "down" to fill the gap
                Array.Copy(array.ToArray(), oldIndex + (dynamic)1, array.ToArray(), (dynamic)oldIndex, newIndex - (dynamic)oldIndex);
            }
            array[(dynamic)newIndex] = tmp;
        }



        public override int[] GetCS_LeadingIndexArray()
        {
            if (!isCompressed)
                (this).Compress();
            return new int[2];//leadIndex;
        }

        public override int[] GetCS_TrailingIndexArray()
        {
            if (!isCompressed)
                (this).Compress();
            return new int[2];//trailIndex;
        }

        public override double[] GetCS_ValueArray()
        {
            if (!isCompressed)
                (this).Compress();
            return values.Keys.ToArray();
        }

    public override void Reset(int row, int col, int nonzeros = 0) { }
        public override bool Resize(int nrows, int ncols, int nonzeros = 0) { return true; }
        public override void SetElement(int row_sel, int col_sel, double insval, bool overwrite = true)
        {
            var lead_sel = row_major_format ? row_sel : col_sel;
            var trail_sel = row_major_format ? col_sel : row_sel;

            if (insval == 0 && !m_lock)
                return;

            int trail_i;
           /* for (trail_i = leadIndex[lead_sel]; trail_i < leadIndex[lead_sel + 1]; ++trail_i)
            {
                // the requested element DOES NOT exist yet, BUT
                // NO other elements with greater index have been stored yet, SO
                // we can just place the new element here
                if (!initialized_element[trail_i])
                {
                    initialized_element[trail_i] = true;
                    trailIndex[trail_i] = trail_sel;
                    values[trail_i] = insval;
                    return;
                }

                // the requested element DOES NOT exist yet AND
                // another element with greater index has already been stored, SO
                // that element has to be pushed further!
                if (trailIndex[trail_i] > trail_sel)
                {
                    // insertion needed
                    break;
                }

                // the requested element already exists
                if (trailIndex[trail_i].Item1 == trail_sel)
                {
                    if (overwrite)
                        values[trail_i] = insval;
                    else
                    {
                        values[trail_i] += insval;
                    }
                    return;
                }
            }*/

            // if the loop ends without break means 'row full' (the insertion will also move the other row, for sure!)

            // insertion needed
           /* insert(ref trail_i, lead_sel);
            initialized_element[trail_i] = true;
            trailIndex[trail_i] = trail_sel;
            values[trail_i] = insval;*/
        }

        /// Returns the value of the element with index (\a row, \a col).
        /// Returns \c zero if an element is not stored.
        public override double GetElement(int row, int col) { return 0; }
    }

}
