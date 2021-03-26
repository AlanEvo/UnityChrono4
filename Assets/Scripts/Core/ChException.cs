using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{

    public class ChException : System.Exception
    {
        protected string m_swhat;

        /// Constructor for a basic exception: sets the exception
        /// message as a string 'swhat'.
        public ChException(string swhat)
        {
            m_swhat = swhat;
        }


    }
}