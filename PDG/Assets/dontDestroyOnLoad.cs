using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class dontDestroyOnLoad : MonoBehaviour 
{
	public Camera maincam;
	public Canvas canvas;

	void Start ()
	{

	}

	void Update()
	{
		maincam = GameObject.Find("Main Camera").GetComponent<Camera>();
		canvas = GameObject.Find("Canvas").GetComponent<Canvas>();

		canvas.worldCamera = maincam;
	}

	void Awake() 
	{
         DontDestroyOnLoad(this);
 
         if (FindObjectsOfType(GetType()).Length > 1)
         {
             Destroy(gameObject);
         }
    }
    
}
