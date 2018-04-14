using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class cameraRender : MonoBehaviour {
    // Use this for initialization

    public bool toggleDelaunay = false;
    public bool toggleHallways = false;
    public bool togglePrims = false;


	void Start () {
        Time.timeScale = 1f;
	}
	
	// Update is called once per frame
	void Update ()
    {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            if(toggleDelaunay)
            {
                toggleDelaunay = false;
            }
            else
            {
                toggleDelaunay = true;
            }
        }

        if (Input.GetKeyDown(KeyCode.W))
        {
            if (togglePrims)
            {
                togglePrims = false;
            }
            else
            {
                togglePrims = true;
            }
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            if (toggleHallways)
            {
                toggleHallways = false;
            }
            else
            {
                toggleHallways = true;
            }
        }

    }

	void OnPostRender() 
	{
        if(toggleDelaunay)
        {
            StartCoroutine(drawDelaunay());
        }

        if(toggleHallways)
        {
            StartCoroutine(PDG.drawHallways());
        }

        if(togglePrims)
        {
            StartCoroutine(PDG.drawPrim());
        }
    }


    IEnumerator drawDelaunay()
    {
        foreach(Triangle tr in PDG.delaunay)
        {
            tr.draw();
            yield return new WaitForEndOfFrame();
        }
    }

}
