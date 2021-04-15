using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
	//////////////////////////////////////////////////////////////////
	//
	// GENERIC LISTS AND NODES
	// ***OBSOLETE***

	/// Node for linked list.
	/// This will be used by ChList<>	
	public unsafe class ChNode<dt> where dt: notnull
	{
		public dt[] data; //< pointer to data
		public ChNode<dt> prev; //< previous node
		public ChNode<dt> next; //< next node

		/// Constructor for node
		public ChNode()
		{
			data = null;
			next = prev = null;
		}

		/// Constructor for node
		public ChNode(dt[] mydata)
		{
			data = mydata;
			next = prev = null;
		}

		//~Ch_node();
	}

	///
	/// Class for linked list.
	///
	/// This linked list class can be used to
	/// build lists of pointers to objects.
	/// This has a different meaning from STL
	/// lists (which are lists of objects), because ours
	/// can manage the deletion of all pointed objects.
	///

	public class ChList<dt> where dt: notnull// : System.IDisposable
	{
		private ChNode<dt> head;
		private ChNode<dt> tail;


		public ChList() { }
		/// Deletion

		public void Dispose() { }

		/// Returns the data at the head of the list
		public dt[] GetHeadData()
		{
			if (head != null)
			{
				return head.data;
			}
			else
			{
				return null;
			}
		}

		/// Returns the data at the tail of the list
		public dt[] GetTailData()
		{
			if (tail != null)
			{
				return tail.data;
			}
			else
			{
				return null;
			}
		}

		/// Returns the head node
		public ChNode<dt> GetHead()
		{
			return head;
		}

		/// Returns the tail node
		public ChNode<dt> GetTail()
		{
			return tail;
		}

		/// Returns a node at given position in list. Returns null
		/// if num exceeds num of nodes. Note: num=1 gets first	element,
		public ChNode<dt> GetNum(int num) { return new ChNode<dt>(); }

		/// Insert data at the head of the list
		public void AddHead(dt[] mdata) {
			ChNode<dt> nnode = new ChNode<dt>();
			nnode.data = mdata;
			nnode.prev = null;
			nnode.next = head;
			if (head == null)  // first elem
			{
				head = nnode;
				tail = nnode;
			}
			else
			{
				head.prev = nnode;
				head = nnode;
			}
		}

		/// Insert data at the tail of the list
		public void AddTail(dt[] mdata) {
			ChNode<dt> nnode = new ChNode<dt>();
			nnode.data = mdata;
			nnode.prev = tail;
			nnode.next = null;
			if (head == null)  // first elem
			{
				head = nnode;
				tail = nnode;
			}
			else
			{
				tail.next = nnode;
				tail = nnode;
			}
		}

		/// Removes the head of list.
		public bool RemHead() { return true; }

		/// Removes the tail of list.
		public bool RemTail() { return true; }

		public void InsertAfter(ChNode<dt> mnode, ChNode<dt> newnode) { }

		public void InsertBefore(ChNode<dt> mnode, ChNode<dt> newnode) { }

		public void InsertAfter(ChNode<dt> mnode, dt mdata) { }

		public void InsertBefore(ChNode<dt> mnode, dt mdata) { }

		/// Removes a node.
		/// Note the Remove() command delete just the Ch_node, not the pointed 'data' object

		public void Remove(ChNode<dt> mnode) { }

		/// Removes all nodes.		
		public void RemoveAll() { }

		/// Removes a node and deletes its data.
		/// Note the Kill() commands remove and also use delete(data) to free the objects pointed by nodes!,
		public void Kill(ChNode<dt> mnode) { }

		/// Kills all nodes.
		public void KillAll() { }

		/// Returns number of elements in list.
		public int Count() { return 0; }

	}
}
