using System.Collections;
using System.Collections.Generic;
using UnityEngine;
/*
namespace chrono {

    public class SceneController : MonoBehaviour {
        private Camera camera;
        private ChSystem m_system;

        // Use this for initialization
        void Start() {
            m_system = GameObject.FindObjectOfType<ChSystem>();
            camera = Camera.current.GetComponent<Camera>();
        }

        // Update is called once per frame
        void Update() {
            camera = Camera.current.GetComponent<Camera>();

             if (Input.GetMouseButtonDown(0))
             {
                 RaycastHit hit;
                 Ray ray = camera.ScreenPointToRay(Input.mousePosition);

                 if (Physics.Raycast(ray))
                 {
                     ChVector from = new ChVector(ray.origin.x, ray.origin.y, 1000);
                     ChVector to = new ChVector(ray.origin.x, ray.origin.y, -1000);
                    // ChVector target = new ChVector(hit.transform.position.x, hit.transform.position.y, hit.transform.position.z);
                     collision.ChRayhitResult mresult = new collision.ChRayhitResult();
                     m_system.GetCollisionSystem().RayHit(from, to, mresult);
                     if (mresult.hit)
                     {
                         if (ChBody mbo = dynamic_cast<ChBody*>(mresult.hitModel.GetContactable()))
                         {
                             app.selectedmover = new std::shared_ptr<ChBody>(mbo);
                             app.selectedpoint = (*(app.selectedmover)).Point_World2Body(mresult.abs_hitPoint);
                             app.selecteddist = (mfrom - mresult.abs_hitPoint).Length();
                             app.selectedspring = new std::shared_ptr<ChLinkSpring>(new ChLinkSpring);
                             app.selectedtruss = new std::shared_ptr<ChBody>(new ChBody);
                             (*(app.selectedtruss)).SetBodyFixed(true);
                             app.GetSystem().AddBody(*(app.selectedtruss));
                             (*(app.selectedspring))
                                 .Initialize(*app.selectedtruss, *app.selectedmover, false, mresult.abs_hitPoint,
                                     mresult.abs_hitPoint);
                             app.GetSystem().AddLink(*(app.selectedspring));
                         }
                     }
                 }
             }
         }
        }
    }*/
