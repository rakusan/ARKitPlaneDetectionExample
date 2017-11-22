using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.iOS;

public class PlaneDetectionExample : MonoBehaviour {

	[SerializeField]
	private GameObject planePrefab;

	private System.Random random = new System.Random();

	private Vector3[] pointCloudData;
	private bool frameUpdated;

	void Start () {
		UnityARSessionNativeInterface.ARFrameUpdatedEvent += ARFrameUpdated;
		frameUpdated = false;
	}

	private void ARFrameUpdated(UnityARCamera camera) {
		pointCloudData = camera.pointCloudData;
		frameUpdated = true;
	}

	void Update () {
		if (frameUpdated) {
			DetectPlane ();
			frameUpdated = false;
		}
	}

	private void DetectPlane() {
		Ransac ();
	}

	private void Ransac() {
		if (pointCloudData.Length < 10) {
			return;
		}

		int kmax = 200;
		float threshold = 0.02f;

		int inlierMax = 0;
		Vector3 tmpPlanePoint = Vector3.zero;
		Vector3 tmpPlaneNormal = Vector3.zero;

		for (int k = 0; k < kmax; ++k) {
			int i0, i1, i2;
			Next3Randoms (pointCloudData.Length, out i0, out i1, out i2);

			Vector3 p0 = pointCloudData [i0];
			Vector3 p1 = pointCloudData [i1];
			Vector3 p2 = pointCloudData [i2];
			Vector3 v1 = p1 - p0;
			Vector3 v2 = p2 - p0;
			Vector3 normal = Vector3.Cross (v1, v2).normalized;

			// 平面の式
			// Dot(normal, p-p0) = 0, ここでpは平面上の任意の点
			//
			// 平面と点qの距離
			// distance = | Dot(normal, q-p0) |
			//   		= | Dot(normal, q) - Dot(normal, p0) |

			float dot_normal_p0 = Vector3.Dot(normal, p0);

			int inlierCount = 0;
			foreach (Vector3 p in pointCloudData) {
				float distance = Mathf.Abs (Vector3.Dot (normal, p) - dot_normal_p0);
				if (distance < threshold) {
					++inlierCount;
				}
			}
			if (inlierCount > inlierMax) {
				inlierMax = inlierCount;
				tmpPlanePoint = p0;
				tmpPlaneNormal = normal;
			}
		}

		if (inlierMax >= pointCloudData.Length / 4) {
			// 平面がカメラの方を向いている必要がある場合はチェックして反転する。
			//if (Vector3.Dot (tmpPlaneNormal, Camera.main.transform.position - tmpPlanePoint) < 0) {
			//	tmpPlaneNormal = -tmpPlaneNormal;
			//}

			GameObject plane = Instantiate (planePrefab, tmpPlanePoint, Quaternion.LookRotation (tmpPlaneNormal));
			Destroy (plane, 3);
		}
	}

	private void Next3Randoms(int maxValue, out int i1, out int i2, out int i3) {
		i1 = random.Next (maxValue);

		do {
			i2 = random.Next (maxValue);
		} while (i2 == i1);

		do {
			i3 = random.Next (maxValue);
		} while (i3 == i1 || i3 == i2);
	}
}
