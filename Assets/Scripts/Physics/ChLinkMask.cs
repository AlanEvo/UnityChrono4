using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Mask structure for N scalar constraint equations between two bodies.
    public class ChLinkMask
    {

        protected List<ChConstraintTwoBodies> constraints = new List<ChConstraintTwoBodies>();  //< array of pointers to 'n' scalar constraint states

        public int nconstr;  //< number of scalar eq.of constraint.

        /// Build a link mask with a single constraint of class ChConstraintTwoBodies().
        public ChLinkMask()
        {
            nconstr = 1;
            constraints.Resize(1);
            constraints[0] = new ChConstraintTwoBodies();
        }

        /// Build a link mask with a default array of mnconstr constraints
        /// of class ChConstraintTwoBodies().
        public ChLinkMask(int mnconstr)
        {
            nconstr = mnconstr;

            constraints.Resize(nconstr);
            for (int i = 0; i < nconstr; i++)
                constraints[i] = new ChConstraintTwoBodies();
        }

        /// Copy constructor
        public ChLinkMask(ChLinkMask other)
        {
            nconstr = other.nconstr;
            constraints.Resize(other.nconstr);
            for (int i = 0; i < nconstr; i++)
                constraints[i] = (ChConstraintTwoBodies)other.constraints[i].Clone();
        }

        /// "Virtual" copy constructor (covariant return type).
        public virtual ChLinkMask Clone() { return new ChLinkMask(this); }

        /// Set references to variables of two connected bodies to all
        /// constraints at once, therefore also sets all the constraints as active.
        public void SetTwoBodiesVariables(ChVariables var1, ChVariables var2)
        {
            for (int i = 0; i < nconstr; i++)
                constraints[i].SetVariables(var1, var2);
        }

        /// Obtain the reference to the i-th scalar constraint data
        /// in the collection link mask.
        public ChConstraintTwoBodies Constr_N(int i)
        {
           // Debug.Assert((i >= 0) && (i < nconstr));
            return constraints[i];
        }

        /// Utility: to change the size of the mask, reallocating constraints
        /// (all of type ChConstraintTwo).
        /// No action if newnconstr == nconstr
        public void ResetNconstr(int newnconstr)
        {
            if (nconstr != newnconstr)
            {
                int i;
                for (i = 0; i < nconstr; i++)
                    if (constraints[i] == null)
                        constraints[i] = null;

                nconstr = newnconstr;

                constraints.Resize(nconstr);

                for (i = 0; i < nconstr; i++)
                    constraints[i] = new ChConstraintTwoBodies();
            }
        }

        /// Add a ChConstraintTwoBodies to mask (NOTE: later, the constraint will
        /// be automatically deleted when the mask will be deleted)
        public void AddConstraint(ChConstraintTwoBodies aconstr)
        {
            nconstr++;
            constraints.Add(aconstr);
        }

        /// To compare two masks, return true if equal
        public bool IsEqual(ChLinkMask mask2)
        {
            if (nconstr != mask2.nconstr)
                return false;
            for (int j = 0; j < nconstr; j++)
            {
                if (!(Constr_N(j) == mask2.Constr_N(j)))
                    return false;
            }
            return true;
        }

        /// Tells if i-th equation is a unilateral constraint
        public bool IsUnilateral(int i)
        {
            if (Constr_N(i).IsUnilateral())
                return true;
            return false;
        }

        // Get the number of removed degrees of freedom (n.of constraints)

        /// Count both bilaterals and unilaterals
        public int GetMaskDoc()
        {
            int tot = 0;
            for (int j = 0; j < nconstr; j++)
            {
                if (Constr_N(j).IsActive())
                    tot++;
            }
            return tot;
        }

        /// Count only unilaterals
        public int GetMaskDoc_d()
        {
            int cnt = 0;
            for (int i = 0; i < nconstr; i++)
            {
                if (Constr_N(i).IsActive())
                    if (this.IsUnilateral(i))  // if (Constr_N(i).IsUnilateral())  BETTER?
                        cnt++;
            }
            return cnt;
        }

        /// Count only bilaterals
        public int GetMaskDoc_c()
        {
            return (GetMaskDoc() - GetMaskDoc_d());
        }

        /// Get the i-th active scalar constraint (not active constr. won't be considered)
        public ChConstraintTwoBodies GetActiveConstrByNum(int mnum)
        {
            int cnt = 0;
            for (int i = 0; i < nconstr; i++)
            {
                if (Constr_N(i).IsActive())
                {
                    if (cnt == mnum)
                        return Constr_N(i);
                    cnt++;
                }
            }
            return null;
        }

        /// Sets some active constraints as redundant.
        public int SetActiveRedundantByArray(int[] mvector, int mcount)
        {
            int cnt;

            ChLinkMask newmask = Clone();
            for (int elem = 0; elem < mcount; elem++)
            {
                cnt = 0;
                for (int i = 0; i < nconstr; i++)
                {
                    if (constraints[i].IsActive())
                    {
                        if (cnt == mvector[elem])
                            newmask.constraints[i].SetRedundant(true);
                        cnt++;
                    }
                }
            }

            // Replace the mask with updated one.
            for (int i = 0; i < nconstr; i++)
            {
                constraints[i] = null;
                constraints[i] = (ChConstraintTwoBodies)newmask.constraints[i].Clone();
            }

            return mcount;
        }

        /// Set lock =ON for constraints which were disabled because redundant
        public int RestoreRedundant()
        {
            int tot = 0;
            for (int j = 0; j < nconstr; j++)
            {
                if (Constr_N(j).IsRedundant())
                {
                    Constr_N(j).SetRedundant(false);
                    tot++;
                }
            }
            return tot;
        }

        /// If SetAllDisabled(true), all the constraints are temporarily turned
        /// off (inactive) at once, because marked as 'disabled'. Return n.of changed
        public int SetAllDisabled(bool mdis)
        {
            int cnt = 0;

            for (int i = 0; i < nconstr; i++)
            {
                if (Constr_N(i).IsDisabled() != mdis)
                {
                    Constr_N(i).SetDisabled(mdis);
                    cnt++;
                }
            }

            return cnt;
        }

        /// If SetAllBroken(true), all the constraints are temporarily turned
        /// off (broken) at once, because marked as 'broken'. Return n.of changed.
        public int SetAllBroken(bool mdis)
        {
            int cnt = 0;

            for (int i = 0; i < nconstr; i++)
            {
                if (Constr_N(i).IsBroken() != mdis)
                {
                    Constr_N(i).SetBroken(mdis);
                    cnt++;
                }
            }

            return cnt;
        }

    }

    /// Specialized ChLinkMask class, for constraint equations of
    /// the ChLinkLock link.
    public class ChLinkMaskLF : ChLinkMask
    {

        /// Create a ChLinkMaskLF which has 7 scalar constraints of
        /// class ChConstraintTwoBodies(). This is useful in case it must
        /// be used for the ChLinkLock link.
        public ChLinkMaskLF()
        {
            ResetNconstr(7);  // the LF formulation uses 7 constraint flags
        }

        public ChLinkMaskLF(ChLinkMaskLF other) : base(other) { }

        /// "Virtual" copy constructor (covariant return type).
        public override ChLinkMask Clone() { return new ChLinkMaskLF(this); }

        /// Set all mask data at once
        public void SetLockMask(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3)
        {
            if (x)
                Constr_X().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_X().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            if (y)
                Constr_Y().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_Y().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            if (z)
                Constr_Z().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_Z().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            if (e0)
                Constr_E0().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_E0().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            if (e1)
                Constr_E1().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_E1().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            if (e2)
                Constr_E2().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_E2().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            if (e3)
                Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
            else
                Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
        }

        /// Obtain the reference to specific scalar constraint data
        /// in the collection of this link mask.
        public ChConstraintTwoBodies Constr_X() { return constraints[0]; }
        public ChConstraintTwoBodies Constr_Y() { return constraints[1]; }
        public ChConstraintTwoBodies Constr_Z() { return constraints[2]; }
        public ChConstraintTwoBodies Constr_E0() { return constraints[3]; }
        public ChConstraintTwoBodies Constr_E1() { return constraints[4]; }
        public ChConstraintTwoBodies Constr_E2() { return constraints[5]; }
        public ChConstraintTwoBodies Constr_E3() { return constraints[6]; }

    };
}
