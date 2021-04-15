using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Class for assemblies of items, for example ChBody, ChLink, ChMesh, etc.
    /// Note that an assembly can be added to another assembly, to create a tree-like hierarchy.
    /// All positions of rigid bodies, FEA nodes, etc. are assumed with respect to the absolute frame.
    public class ChAssembly : ChPhysicsItem
    {
        public ChAssembly()
        {
            nbodies = 0;
            nlinks = 0;
            nmeshes = 0;
            nphysicsitems = 0;
            ndof = 0;
            ndoc = 0;
            ndoc_w = 0;
            ndoc_w_C = 0;
            ndoc_w_D = 0;
            ncoords = 0;
            ncoords_w = 0;
            nsysvars = 0;
            nsysvars_w = 0;
            nbodies_sleep = 0;
            nbodies_fixed = 0;
        }
        public ChAssembly(ChAssembly other)
        {
            nbodies = other.nbodies;
            nlinks = other.nlinks;
            nmeshes = other.nmeshes;
            nphysicsitems = other.nphysicsitems;
            ndof = other.ndof;
            ndoc = other.ndoc;
            ndoc_w = other.ndoc_w;
            ndoc_w_C = other.ndoc_w_C;
            ndoc_w_D = other.ndoc_w_D;
            ncoords = other.ncoords;
            ncoords_w = other.ncoords_w;
            nsysvars = other.nsysvars;
            nsysvars_w = other.nsysvars_w;
            nbodies_sleep = other.nbodies_sleep;
            nbodies_fixed = other.nbodies_fixed;
        }

        /// "Virtual" copy ructor (covariant return type).
        public override ChObj Clone() { return new ChAssembly(this); }

        //
        // CONTAINER FUNCTIONS
        //

        /// Removes all inserted items: bodies, links, etc.
        public void Clear()
        {
            // RemoveAllLinks();
            // RemoveAllBodies();
            // RemoveAllMeshes();
            // RemoveAllOtherPhysicsItems();

            nbodies = 0;
            nlinks = 0;
            nmeshes = 0;
            nphysicsitems = 0;
            ndof = 0;
            ndoc = 0;
            ndoc_w = 0;
            ndoc_w_C = 0;
            ndoc_w_D = 0;
            ncoords = 0;
            ncoords_w = 0;
            nsysvars = 0;
            nsysvars_w = 0;
            nbodies_sleep = 0;
            nbodies_fixed = 0;
        }

        // Do not add the same item multiple times; also, do not remove items which haven't ever been added!
        // This will most often cause an assert() failure in debug mode.
        // Note. adding/removing items to the assembly doesn't call Update() automatically.

        /// Attach a body to this assembly.
        public virtual void AddBody(ChBody body)
        {
            // Debug.Assert(std::find(std::begin(bodylist), std::end(bodylist), body) == bodylist.end());
            //  Debug.Assert(body.GetSystem() == null);  // should remove from other system before adding here

            // set system and also add collision models to system
            body.SetSystem(system);
            bodylist.Add(body);
        }

        /// Attach a link to this assembly.
        public virtual void AddLink(ChLink link)
        {
            link.SetSystem(system);
            linklist.Add(link);
        }

        /// Attach a mesh to this assembly.
      //  virtual void AddMesh(std::shared_ptr<fea::ChMesh> mesh);

        /// Attach a ChPhysicsItem object that is not a body, link, or mesh.
        public virtual void AddOtherPhysicsItem(ChPhysicsItem item)
        {

        }

        /// Attach an arbitrary ChPhysicsItem (e.g. ChBody, ChParticles, ChLink, etc.) to the assembly.
        /// It will take care of adding it to the proper list of bodies, links, meshes, or generic
        /// physic item. (i.e. it calls AddBody(), AddLink(), AddMesh(), or AddOtherPhysicsItem()).
        /// Note, you cannot call Add() during an Update (i.e. items like particle generators that
        /// are already inserted in the assembly cannot call this) because not thread safe; instead,
        /// use AddBatch().
        public void Add(ChPhysicsItem item, int key)
        {
            ChBody body = new ChBody();
            if (body == (ChBody)item)
            {
                body = (ChBody)item;
                Add(body, key);
                return;
            }

            /* if (auto link = std::dynamic_pointer_cast<ChLink>(item)) {
                 AddLink(link);
                 return;
             }

             if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
                 AddMesh(mesh);
                 return;
             }*/

            AddOtherPhysicsItem(item);
        }

        //
        // STATISTICS
        //

        /// Get the number of active bodies (so, excluding those that are sleeping or are fixed to ground).
        public int GetNbodies() { return nbodies; }
        /// Get the number of bodies that are in sleeping mode (excluding fixed bodies).
        public int GetNbodiesSleeping() { return nbodies_sleep; }
        /// Get the number of bodies that are fixed to ground.
        public int GetNbodiesFixed() { return nbodies_fixed; }
        /// Get the total number of bodies added to the assembly, including the grounded and sleeping bodies.
        public int GetNbodiesTotal() { return nbodies + nbodies_fixed + nbodies_sleep; }

        /// Get the number of links.
        public int GetNlinks() { return nlinks; }

        /// Get the number of meshes.
        public int GetNmeshes() { return nmeshes; }

        /// Get the number of other physics items (other than bodies, links, or meshes).
        public int GetNphysicsItems() { return nphysicsitems; }

        /// Get the number of coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions).
        public int GetNcoords() { return ncoords; }
        /// Get the number of degrees of freedom of the assembly.
        public int GetNdof() { return ndof; }
        /// Get the number of scalar raints added to the assembly, including raints on quaternion norms.
        public int GetNdoc() { return ndoc; }
        /// Get the number of system variables (coordinates plus the raint multipliers, in case of quaternions).
        public int GetNsysvars() { return nsysvars; }
        /// Get the number of coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
        public int GetNcoords_w() { return ncoords_w; }
        /// Get the number of scalar raints added to the assembly.
        public int GetNdoc_w() { return ndoc_w; }
        /// Get the number of scalar raints added to the assembly (only bilaterals).
        public int GetNdoc_w_C() { return ndoc_w_C; }
        /// Get the number of scalar raints added to the assembly (only unilaterals).
        public int GetNdoc_w_D() { return ndoc_w_D; }
        /// Get the number of system variables (coordinates plus the raint multipliers).
        public int GetNsysvars_w() { return nsysvars_w; }

        //
        // PHYSICS ITEM INTERFACE
        //

        /// Set the pointer to the parent ChSystem() and
        /// also add to new collision system / remove from old coll.system
        public override void SetSystem(ChSystem m_system)
        {
            system = m_system;

            for(int i = 0; i < bodylist.Count; i++)
            {
                bodylist[i].SetSystem(m_system);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].SetSystem(m_system);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->SetSystem(m_system);
            }
            for (auto & item : otherphysicslist)
            {
                item->SetSystem(m_system);
            }*/
        }

        public void FlushBatch()
        {
            /*  foreach (ChPhysicsItem item in batch_to_insert)
              {
                  Add(item);
              }*/
            batch_to_insert.Clear();
        }

        public override void SyncCollisionModels()
        {
            for (int i = 0; i < bodylist.Count; i++)
            {
                bodylist[i].SyncCollisionModels();
            }
            for (int i = 0; i < linklist.Count; i++)
            {
                linklist[i].SyncCollisionModels();
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->SyncCollisionModels();
            }
            for (auto & item : otherphysicslist)
            {
                item->SyncCollisionModels();
            }*/
        }

        /// Counts the number of bodies, links, and meshes.
        /// Computes the offsets of object states in the global state.
        /// Assumes that this->offset_x this->offset_w this->offset_L are already set
        /// as starting point for offsetting all the contained sub objects.
        public override void Setup()
        {
            nbodies = 0;
            nbodies_sleep = 0;
            nbodies_fixed = 0;
            ncoords = 0;
            ncoords_w = 0;
            ndoc = 0;
            ndoc_w = 0;
            ndoc_w_C = 0;
            ndoc_w_D = 0;
            nlinks = 0;
            nmeshes = 0;
            nphysicsitems = 0;

            // Add any items queued for insertion in the assembly's lists.
            this.FlushBatch();

            for (int i = 0; i < bodylist.Count; i++)
            {
                if (bodylist[i].GetBodyFixed())
                    nbodies_fixed++;
                else if (bodylist[i].GetSleeping())
                    nbodies_sleep++;
                else
                {
                    nbodies++;

                    bodylist[i].SetOffset_x(this.offset_x + ncoords);
                    bodylist[i].SetOffset_w(this.offset_w + ncoords_w);
                    bodylist[i].SetOffset_L(this.offset_L + ndoc_w);

                    // body->Setup(); // not needed since in bodies does nothing

                    ncoords += bodylist[i].GetDOF();
                    ncoords_w += bodylist[i].GetDOF_w();
                    ndoc_w += bodylist[i].GetDOC();      // not really needed since ChBody introduces no constraints
                    ndoc_w_C += bodylist[i].GetDOC_c();  // not really needed since ChBody introduces no constraints
                    ndoc_w_D += bodylist[i].GetDOC_d();  // not really needed since ChBody introduces no constraints
                }
            }

            for (int i = 0; i < linklist.Count; i++)
            {
                if (linklist[i].IsActive())
                {
                    nlinks++;

                    linklist[i].SetOffset_x(this.offset_x + ncoords);
                    linklist[i].SetOffset_w(this.offset_w + ncoords_w);
                    linklist[i].SetOffset_L(this.offset_L + ndoc_w);

                    linklist[i].Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

                    ncoords += linklist[i].GetDOF();
                    ncoords_w += linklist[i].GetDOF_w();
                    ndoc_w += linklist[i].GetDOC(); 

                    ndoc_w_C += linklist[i].GetDOC_c();
                    ndoc_w_D += linklist[i].GetDOC_d(); 
                }
            }

            ndoc = ndoc_w + nbodies;          // number of constraints including quaternion constraints.
            nsysvars = ncoords + ndoc;        // total number of variables (coordinates + lagrangian multipliers)
            nsysvars_w = ncoords_w + ndoc_w;  // total number of variables (with 6 dof per body)

            // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)
            ndof = ncoords_w - ndoc_w;
        }

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state.
        public override void update(double mytime, bool update_assets = true)
        {
            base.update(mytime, update_assets);
            update(update_assets);
        }

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state.
        public override void update(bool update_assets = true)
        {
            //// NOTE: do not switch these to range for loops (may want to use OMP for)
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].update(ChTime, update_assets);
            }
            /* for (int ip = 0; ip < (int)otherphysicslist.Count; ++ip)
             {
                 otherphysicslist[ip].update(ChTime, update_assets);
             }*/
            for (int ip = 0; ip < (int)linklist.Count ; ++ip)
            {
                linklist[ip].update(ChTime, update_assets);
            }
            /* for (int ip = 0; ip < (int)meshlist.size(); ++ip)
             {
                 meshlist[ip]->Update(ChTime, update_assets);
             }*/
        }

        /// Set zero speed (and zero accelerations) in state, without changing the position.
        public override void SetNoSpeedNoAcceleration()
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].SetNoSpeedNoAcceleration();
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].SetNoSpeedNoAcceleration();
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->SetNoSpeedNoAcceleration();
            }
            for (auto & item : otherphysicslist)
            {
                item->SetNoSpeedNoAcceleration();
            }*/
        }

        /// Get the number of scalar coordinates (ex. dim of position vector)
        public override int GetDOF() { return GetNcoords(); }
        /// Get the number of scalar coordinates of variables derivatives (ex. dim of speed vector)
        public override int GetDOF_w() { return GetNcoords_w(); }
        /// Get the number of scalar raints, if any, in this item
        public override int GetDOC() { return GetNdoc_w(); }
        /// Get the number of scalar raints, if any, in this item (only bilateral r.)
        public override int GetDOC_c() { return GetNdoc_w_C(); }
        /// Get the number of scalar raints, if any, in this item (only unilateral r.)
        public override int GetDOC_d() { return GetNdoc_w_D(); }

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGather(int off_x,
                                ref ChState x,
                                int off_v,
                                ref ChStateDelta v,
                                ref double T)
        {
            int displ_x = off_x - this.offset_x;
            int displ_v = off_v - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateGather(displ_x + bodylist[i].GetOffset_x(), ref x, displ_v + bodylist[i].GetOffset_w(), ref v, ref T);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntStateGather(displ_x + linklist[i].GetOffset_x(), ref x, displ_v + linklist[i].GetOffset_w(), ref v, ref T);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntStateGather(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntStateGather(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
            }*/
            T = GetChTime();
        }
        public override void IntStateScatter(int off_x,
                                 ChState x,
                                 int off_v,
                                 ChStateDelta v,
                                 double T)
        {
            int displ_x = off_x - this.offset_x;
            int displ_v = off_v - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateScatter(displ_x + bodylist[i].GetOffset_x(), x, displ_v + bodylist[i].GetOffset_w(), v, T);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
               // if (linklist[i].IsActive())
                    linklist[i].IntStateScatter(displ_x + linklist[i].GetOffset_x(), x, displ_v + linklist[i].GetOffset_w(), v, T);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
            }*/
            SetChTime(T);

            // Note: all those IntStateScatter() above should call Update() automatically
            // for each object in the loop, therefore:
            // -do not call Update() on this.
            // -do not call ChPhysicsItem::IntStateScatter() -it calls this->Update() anyway-
            // because this would cause redundant updates.
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a)
        {
            int displ_a = off_a - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateGatherAcceleration(displ_a + bodylist[i].GetOffset_w(), ref a);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntStateGatherAcceleration(displ_a + linklist[i].GetOffset_w(), ref a);
            }
            /*for (auto & mesh : meshlist)
             {
                 mesh->IntStateGatherAcceleration(displ_a + mesh->GetOffset_w(), a);
             }
             for (auto & item : otherphysicslist)
             {
                 item->IntStateGatherAcceleration(displ_a + item->GetOffset_w(), a);
             }*/
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a)
        {
            int displ_a = off_a - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateScatterAcceleration(displ_a + bodylist[i].GetOffset_w(), a);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntStateScatterAcceleration(displ_a + linklist[i].GetOffset_w(), a);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntStateScatterAcceleration(displ_a + mesh->GetOffset_w(), a);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntStateScatterAcceleration(displ_a + item->GetOffset_w(), a);
            }*/
        }
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            int displ_L = off_L - this.offset_L;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateGatherReactions(displ_L + bodylist[i].GetOffset_L(), ref L);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntStateGatherReactions(displ_L + linklist[i].GetOffset_L(), ref L);
            }
            /* for (auto & mesh : meshlist)
             {
                 mesh->IntStateGatherReactions(displ_L + mesh->GetOffset_L(), L);
             }
             for (auto & item : otherphysicslist)
             {
                 item->IntStateGatherReactions(displ_L + item->GetOffset_L(), L);
             }*/
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            int displ_L = off_L - this.offset_L;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateScatterReactions(displ_L + bodylist[i].GetOffset_L(), L);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntStateScatterReactions(displ_L + linklist[i].GetOffset_L(), L);
            }
            /* for (auto & mesh : meshlist)
             {
                 mesh->IntStateScatterReactions(displ_L + mesh->GetOffset_L(), L);
             }
             for (auto & item : otherphysicslist)
             {
                 item->IntStateScatterReactions(displ_L + item->GetOffset_L(), L);
             }*/
        }
        public override void IntStateIncrement(int off_x,
                                   ref ChState x_new,
                                   ChState x,
                                   int off_v,
                                   ChStateDelta Dv)
        {
            int displ_x = off_x - (int)this.offset_x;
            int displ_v = off_v - (int)this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntStateIncrement(displ_x + (int)bodylist[i].GetOffset_x(), ref x_new, x, displ_v + (int)bodylist[i].GetOffset_w(), Dv);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntStateIncrement(displ_x + linklist[i].GetOffset_x(), ref x_new, x, displ_v + linklist[i].GetOffset_w(), Dv);
            }

            /* for (auto & mesh : meshlist)
             {
                 mesh->IntStateIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
             }

             for (auto & item : otherphysicslist)
             {
                 item->IntStateIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
             }*/
        }
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c)
        {
            int displ_v = off - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntLoadResidual_F(displ_v + bodylist[i].GetOffset_w(), ref R, c);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntLoadResidual_F(displ_v + linklist[i].GetOffset_w(), ref R, c);
            }
            /* for (auto & mesh : meshlist)
             {
                 mesh->IntLoadResidual_F(displ_v + mesh->GetOffset_w(), R, c);
             }
             for (auto & item : otherphysicslist)
             {
                 item->IntLoadResidual_F(displ_v + item->GetOffset_w(), R, c);
             }*/
        }
        public override void IntLoadResidual_Mv(int off,
                                    ref ChVectorDynamic<double> R,
                                    ChVectorDynamic<double> w,
                                    double c)
        {
            int displ_v = off - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntLoadResidual_Mv(displ_v + bodylist[i].GetOffset_w(), ref R, w, c);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntLoadResidual_Mv(displ_v + linklist[i].GetOffset_w(), ref R, w, c);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntLoadResidual_Mv(displ_v + mesh->GetOffset_w(), R, w, c);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntLoadResidual_Mv(displ_v + item->GetOffset_w(), R, w, c);
            }*/
        }
        public override void IntLoadResidual_CqL(int off_L,
                                     ref ChVectorDynamic<double> R,
                                     ChVectorDynamic<double> L,
                                     double c)
        {
            int displ_L = off_L - this.offset_L;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntLoadResidual_CqL(displ_L + bodylist[i].GetOffset_L(), ref R, L, c);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntLoadResidual_CqL(displ_L + linklist[i].GetOffset_L(), ref R, L, c);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntLoadResidual_CqL(displ_L + mesh->GetOffset_L(), R, L, c);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntLoadResidual_CqL(displ_L + item->GetOffset_L(), R, L, c);
            }*/
        }
        public override void IntLoadConstraint_C(int off_L,
                                     ref ChVectorDynamic<double> Qc,
                                     double c,
                                     bool do_clamp,
                                     double recovery_clamp)
        {
            int displ_L = off_L - this.offset_L;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntLoadConstraint_C(displ_L + bodylist[i].GetOffset_L(), ref Qc, c, do_clamp, recovery_clamp);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntLoadConstraint_C(displ_L + linklist[i].GetOffset_L(), ref Qc, c, do_clamp, recovery_clamp);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntLoadConstraint_C(displ_L + mesh->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntLoadConstraint_C(displ_L + item->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
            }*/
        }
        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c)
        {
            int displ_L = off_L - this.offset_L;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntLoadConstraint_Ct(displ_L + bodylist[i].GetOffset_L(), ref Qc, c);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntLoadConstraint_Ct(displ_L + linklist[i].GetOffset_L(), ref Qc, c);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->IntLoadConstraint_Ct(displ_L + mesh->GetOffset_L(), Qc, c);
            }
            for (auto & item : otherphysicslist)
            {
                item->IntLoadConstraint_Ct(displ_L + item->GetOffset_L(), Qc, c);
            }*/
        }
        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc)
        {
            int displ_L = off_L - this.offset_L;
            int displ_v = off_v - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntToDescriptor(displ_v + bodylist[i].GetOffset_w(), v, R, displ_L + bodylist[i].GetOffset_L(), L, Qc);
            }

            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntToDescriptor(displ_v + linklist[i].GetOffset_w(), v, R, displ_L + linklist[i].GetOffset_L(), L, Qc);
            }

            /* for (auto & mesh : meshlist)
             {
                 mesh->IntToDescriptor(displ_v + mesh->GetOffset_w(), v, R, displ_L + mesh->GetOffset_L(), L, Qc);
             }

             for (auto & item : otherphysicslist)
             {
                 item->IntToDescriptor(displ_v + item->GetOffset_w(), v, R, displ_L + item->GetOffset_L(), L, Qc);
             }*/
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L)
        {
            int displ_L = off_L - this.offset_L;
            int displ_v = off_v - this.offset_w;

            for (int i = 0; i < bodylist.Count ; i++)
            {
                if (bodylist[i].IsActive())
                    bodylist[i].IntFromDescriptor(displ_v + bodylist[i].GetOffset_w(), ref v, displ_L + bodylist[i].GetOffset_L(), ref L);
            }

            for (int i = 0; i < linklist.Count ; i++)
            {
                if (linklist[i].IsActive())
                    linklist[i].IntFromDescriptor(displ_v + linklist[i].GetOffset_w(), ref v, displ_L + linklist[i].GetOffset_L(), ref L);
            }

            /* for (auto & mesh : meshlist)
             {
                 mesh->IntFromDescriptor(displ_v + mesh->GetOffset_w(), v, displ_L + mesh->GetOffset_L(), L);
             }

             for (auto & item : otherphysicslist)
             {
                 item->IntFromDescriptor(displ_v + item->GetOffset_w(), v, displ_L + item->GetOffset_L(), L);
             }*/
        }

        public override void InjectVariables(ref ChSystemDescriptor mdescriptor)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].InjectVariables(ref mdescriptor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].InjectVariables(ref mdescriptor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->InjectVariables(mdescriptor);
            }
            for (auto & item : otherphysicslist)
            {
                item->InjectVariables(mdescriptor);
            }*/
        }

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].InjectConstraints(ref mdescriptor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].InjectConstraints(ref mdescriptor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->InjectConstraints(mdescriptor);
            }
            for (auto & item : otherphysicslist)
            {
                item->InjectConstraints(mdescriptor);
            }*/
        }
        public override void ConstraintsLoadJacobians()
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsLoadJacobians();
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].ConstraintsLoadJacobians();
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->ConstraintsLoadJacobians();
            }
            for (auto & item : otherphysicslist)
            {
                item->ConstraintsLoadJacobians();
            }*/
        }

        public override void InjectKRMmatrices(ref ChSystemDescriptor mdescriptor)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].InjectKRMmatrices(ref mdescriptor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].InjectKRMmatrices(ref mdescriptor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->InjectKRMmatrices(mdescriptor);
            }
            for (auto & item : otherphysicslist)
            {
                item->InjectKRMmatrices(mdescriptor);
            }*/
        }
        public override void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto & item : otherphysicslist)
            {
                item->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }*/
        }

        // Old bookkeeping system - to be removed soon
        public override void VariablesFbReset()
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].VariablesFbReset();
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].VariablesFbReset();
            }
            /* for (auto & mesh : meshlist)
             {
                 mesh->VariablesFbReset();
             }
             for (auto & item : otherphysicslist)
             {
                 item->VariablesFbReset();
             }*/
        }
        public override void VariablesFbLoadForces(double factor = 1)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].VariablesFbLoadForces(factor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].VariablesFbLoadForces(factor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->VariablesFbLoadForces(factor);
            }
            for (auto & item : otherphysicslist)
            {
                item->VariablesFbLoadForces(factor);
            }*/
        }
        public override void VariablesQbLoadSpeed()
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].VariablesQbLoadSpeed();
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].VariablesQbLoadSpeed();
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->VariablesQbLoadSpeed();
            }
            for (auto & item : otherphysicslist)
            {
                item->VariablesQbLoadSpeed();
            }*/
        }
        public override void VariablesFbIncrementMq()
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].VariablesFbIncrementMq();
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].VariablesFbIncrementMq();
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->VariablesFbIncrementMq();
            }
            for (auto & item : otherphysicslist)
            {
                item->VariablesFbIncrementMq();
            }*/
        }
        public override void VariablesQbSetSpeed(double step = 0)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].VariablesQbSetSpeed(step);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].VariablesQbSetSpeed(step);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->VariablesQbSetSpeed(step);
            }
            for (auto & item : otherphysicslist)
            {
                item->VariablesQbSetSpeed(step);
            }*/
        }
        public override void VariablesQbIncrementPosition(double dt_step)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].VariablesQbIncrementPosition(dt_step);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].VariablesQbIncrementPosition(dt_step);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->VariablesQbIncrementPosition(dt_step);
            }
            for (auto & item : otherphysicslist)
            {
                item->VariablesQbIncrementPosition(dt_step);
            }*/
        }
        public override void ConstraintsBiReset()
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsBiReset();
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].ConstraintsBiReset();
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->ConstraintsBiReset();
            }
            for (auto & item : otherphysicslist)
            {
                item->ConstraintsBiReset();
            }*/
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            }
            for (auto & item : otherphysicslist)
            {
                item->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            }*/
        }
        public override void ConstraintsBiLoad_Ct(double factor = 1)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsBiLoad_Ct(factor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].ConstraintsBiLoad_Ct(factor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->ConstraintsBiLoad_Ct(factor);
            }
            for (auto & item : otherphysicslist)
            {
                item->ConstraintsBiLoad_Ct(factor);
            }*/
        }
        public override void ConstraintsBiLoad_Qc(double factor = 1)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsBiLoad_Qc(factor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].ConstraintsBiLoad_Qc(factor);
            }
            /* for (auto & mesh : meshlist)
             {
                 mesh->ConstraintsBiLoad_Qc(factor);
             }
             for (auto & item : otherphysicslist)
             {
                 item->ConstraintsBiLoad_Qc(factor);
             }*/
        }
        public override void ConstraintsFbLoadForces(double factor = 1)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsFbLoadForces(factor);
            }
            for (int i = 0; i < linklist.Count ; i++)
            {
                linklist[i].ConstraintsFbLoadForces(factor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->ConstraintsFbLoadForces(factor);
            }
            for (auto & item : otherphysicslist)
            {
                item->ConstraintsFbLoadForces(factor);
            }*/
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            for (int i = 0; i < bodylist.Count ; i++)
            {
                bodylist[i].ConstraintsFetch_react(factor);
            }
            for (int i = 0; i < linklist.Count; i++)
            {
                linklist[i].ConstraintsFetch_react(factor);
            }
            /*for (auto & mesh : meshlist)
            {
                mesh->ConstraintsFetch_react(factor);
            }
            for (auto & item : otherphysicslist)
            {
                item->ConstraintsFetch_react(factor);
            }*/
        }


        protected List<ChBody> bodylist = new List<ChBody>();                 //< list of rigid bodies
        protected List<ChLink> linklist = new List<ChLink>();                 //< list of joints (links)
        // std::vector<std::shared_ptr<fea::ChMesh>> meshlist;            ///< list of meshes
        protected List<ChPhysicsItem> otherphysicslist = new List<ChPhysicsItem>();  //< list of other physics objects
        protected List<ChPhysicsItem> batch_to_insert = new List<ChPhysicsItem>();   //< list of items to insert at once

        // Statistics:
        protected int nbodies;        //< number of bodies (currently active)
        protected int nlinks;         //< number of links
        protected int nmeshes;        //< number of meshes
        protected int nphysicsitems;  //< number of other physics items
        protected int ncoords;        //< number of scalar coordinates (including 4th dimension of quaternions) for all active bodies
        protected int ndoc;           //< number of scalar raints (including r. on quaternions)
        protected int nsysvars;       //< number of variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
        protected int ncoords_w;      //< number of scalar coordinates when using 3 rot. dof. per body;  for all active bodies
        protected int ndoc_w;         //< number of scalar raints  when using 3 rot. dof. per body;  for all active bodies
        protected int nsysvars_w;     //< number of variables when using 3 rot. dof. per body; i.e. = ncoords_w+ndoc_w
        protected int ndof;           //< number of degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
        protected int ndoc_w_C;       //< number of scalar raints C, when using 3 rot. dof. per body (excluding unilaterals)
        protected int ndoc_w_D;       //< number of scalar raints D, when using 3 rot. dof. per body (only unilaterals)
        protected int nbodies_sleep;  //< number of bodies that are sleeping
        protected int nbodies_fixed;  //< number of bodies that are fixed
    }
}

