using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class MathfTest : MonoBehaviour
{

	public int iterations = 50000;
	/*public void OnGUI()
	{
		GUI.color = Color.black;
		GUILayout.Label("100% means no difference in speed, >100% is faster", GUILayout.Width(400));
		GUILayout.Label("Loop " + Mathfx.loop, GUILayout.Width(100));

		GUILayout.BeginHorizontal();
		GUILayout.Space(5);
		GUI.color = Color.black;
		GUILayout.Label("Name (type)", GUILayout.Width(100));
		GUILayout.Label("Last speed", GUILayout.Width(100));
		GUILayout.Label("Average speed", GUILayout.Width(100));
		GUILayout.EndHorizontal();

		for (int i = 0; i < Mathfx.labels.Length; i++)
		{
			GUILayout.BeginHorizontal();
			GUILayout.Space(5);
			GUI.color = Color.black;
			GUILayout.Label(Mathfx.labels[i], GUILayout.Width(100));
			GUI.color = Mathfx.faster[i] ? new Color(0, 0.7F, 0) : new Color(0.7F, 0, 0);
			GUILayout.Label(Mathfx.percent[i] + "%", GUILayout.Width(100));
			GUI.color = Mathfx.avgPercent[i] >= 100 ? new Color(0, 0.7F, 0) : new Color(0.7F, 0, 0);
			GUILayout.Label("avg " + Mathfx.avgPercent[i].ToString("0") + "%", GUILayout.Width(100));

			GUILayout.EndHorizontal();
		}

	}*/

	// Use this for initialization
	/*IEnumerator Start()
	{

		Mathfx.labels = new string[15];
		Mathfx.faster = new bool[15];
		Mathfx.percent = new string[15];
		Mathfx.avgPercent = new double[15];

		yield return new WaitForSeconds(0.5F);

		while (true)
		{
			Mathfx.loop++;

			float ext = Mathf.Pow(10, Random.value * 5);
			float ext2 = Mathf.Pow(10, Random.value * 5);

			float a = Random.value * ext * 2 - ext;
			float b = Random.value * ext2 * 2 - ext2;
			float c = Random.value * ext2 * 2 - ext2;

			int ai = Mathf.RoundToInt(Random.value * 10 - 5);
			int bi = Mathf.RoundToInt(Random.value * 10 - 5);
			int ci = Mathf.RoundToInt(Random.value * 10 - 5);
			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Abs(a);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Abs(a);
			}
			Mathfx.EndTimer2("Abs (float)", 0);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Abs(ai);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Abs(ai);
			}
			Mathfx.EndTimer2("Abs (int)", 1);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Min(a, b);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Min(a, b);
			}
			Mathfx.EndTimer2("Min (float)", 2);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Min(ai, bi);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Min(ai, bi);
			}
			Mathfx.EndTimer2("Min (int)", 3);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Max(a, b);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Max(a, b);
			}
			Mathfx.EndTimer2("Max (float)", 4);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Max(ai, bi);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Max(ai, bi);
			}
			Mathfx.EndTimer2("Max (int)", 5);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Sign(a);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Sign(a);
			}
			Mathfx.EndTimer2("Sign (float)", 6);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Sign(ai);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Sign(ai);
			}
			Mathfx.EndTimer2("Sign (int)", 7);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Clamp(a, b, c);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Clamp(a, b, c);
			}
			Mathfx.EndTimer2("Clamp (float)", 8);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Clamp(ai, bi, ci);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Clamp(ai, bi, ci);
			}
			Mathfx.EndTimer2("Clamp (int)", 9);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Clamp01(a);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Clamp01(a);
			}
			Mathfx.EndTimer2("Clamp01 (float)", 10);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Clamp01(ai);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Clamp01(ai);
			}
			Mathfx.EndTimer2("Clamp01 (int)", 11);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Lerp(a, b, c);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Lerp(a, b, c);
			}
			Mathfx.EndTimer2("Lerp (float)", 12);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Approximately(a, b);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Approximately(a, b);
			}
			Mathfx.EndTimer2("Approximately (float)", 13);
			yield return 0;

			//---------------------
			Mathfx.StartTimer();
			for (int i = 0; i < iterations; i++)
			{
				Mathf.Repeat(a, b);
			}
			Mathfx.EndTimer1();
			for (int i = 0; i < iterations; i++)
			{
				Mathfx.Repeat(a, b);
			}
			Mathfx.EndTimer2("Repeat (float)", 14);
			yield return 0;
		}
	}*/
}

public class Mathfx
{

	static double st;
	static double st2;
	static double st3;

	public static string[] labels;
	public static bool[] faster;
	public static string[] percent;
	public static double[] avgPercent;

	public static int loop = -1;

	public static void StartTimer()
	{
		st = Time.realtimeSinceStartup;
	}
	public static void EndTimer1()
	{
		st2 = Time.realtimeSinceStartup;
	}
	public static void EndTimer2(string label, int index)
	{
		st3 = Time.realtimeSinceStartup;

		st = st2 - st;
		st2 = st3 - st2;

		label = label.PadRight(20);
		labels[index] = label;
		faster[index] = st2 < st;
		double cpercent = Math.Abs((st / st2) * 100);
		percent[index] = cpercent.ToString("0");

		avgPercent[index] = ((avgPercent[index] * loop + cpercent) / (loop + 1));
	}

	public static double Abs(double a)
	{
		if (a < 0)
		{
			return -a;
		}
		return a;
	}

	public static int Abs(int a)
	{
		if (a < 0)
		{
			return -a;
		}
		return a;
	}

	public static double Min(double a, double b)
	{
		return a < b ? a : b;
	}

	public static int Min(int a, int b)
	{
		return a < b ? a : b;
	}

	public static double Max(double a, double b)
	{
		return a > b ? a : b;
	}

	public static int Max(int a, int b)
	{
		return a > b ? a : b;
	}

	//Malfunctioning, won't always return the correct values
	public static int Pow(int a, int b)
	{
		if (b == 0)
		{
			return 1;
		}

		for (int i = 0; i < b; i++)
		{
			a *= a;
		}
		return a;
	}

	public static double Sign(float a)
	{
		return a < 0 ? -1 : 1;
	}

	public static int Sign(int a)
	{
		return a < 0 ? -1 : 1;
	}

	public static double Clamp(double a, double b, double c)
	{
		a = a > c ? c : a;
		return a < b ? b : a;
	}

	public static int Clamp(int a, int b, int c)
	{
		a = a > c ? c : a;
		return a < b ? b : a;
	}

	public static float Clamp01(float a)
	{
		a = a > 1 ? 1 : a;
		return a < 0 ? 0 : a;
	}

	public static int Clamp01(int a)
	{
		a = a > 1 ? 1 : a;
		return a < 0 ? 0 : a;
	}

	public static float Lerp(float a, float b, float t)
	{
		return a + (b - a) * Mathf.Clamp01(t);
	}

	public static bool Approximately(float a, float b)
	{
		return a + 0.0000000596F >= b && a - 0.0000000596F <= b;

		//Also faster than the built-in, but a bit slower than the above
		//a -= b;
		//return a <= 0.0000000596F && a >= -0.0000000596F;
	}

	public static float Repeat(float a, float b)
	{
		return a % b;
	}
}
