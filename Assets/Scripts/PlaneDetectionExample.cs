using System.Collections;
using System.Collections.Generic;
using System.Linq;
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
			Ransac ();
			frameUpdated = false;
		}
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
			// 平面と点qの距離（normalの長さは1とする）
			// distance = | Dot(normal, q-p0) |
			//   		= | Dot(normal, q) - Dot(normal, p0) |

			float dot_normal_p0 = Vector3.Dot(normal, p0);
			int inlierCount = pointCloudData.Count (q => Mathf.Abs (Vector3.Dot (normal, q) - dot_normal_p0) < threshold);

			if (inlierCount > inlierMax) {
				inlierMax = inlierCount;
				tmpPlanePoint = p0;
				tmpPlaneNormal = normal;
			}
		}

		if (inlierMax >= pointCloudData.Length / 4) {
			// 採用した仮の平面にうまく合っている点だけ取り出す
			float dot_normal_p0 = Vector3.Dot(tmpPlaneNormal, tmpPlanePoint);
			IEnumerable<Vector3> inliers = pointCloudData.Where (q => Mathf.Abs (Vector3.Dot (tmpPlaneNormal, q) - dot_normal_p0) < threshold);

			LeastSquaresMethod (tmpPlanePoint, tmpPlaneNormal, inliers);
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

	private void LeastSquaresMethod(Vector3 tmpPlanePoint, Vector3 tmpPlaneNormal, IEnumerable<Vector3> inliers) {
		// 座標変換に使うクォータニオン
		Quaternion rotation = Quaternion.FromToRotation (tmpPlaneNormal, Vector3.forward);

		// 最小二乗法を解く行列の要素
		float A = 0, B = 0, C = 0, D = 0, E = 0, F = 0, G = 0, H = 0;
		foreach (Vector3 q in inliers) {
			Vector3 q2 = rotation * q;
			A += q2.x * q2.x;  // Σ(xi^2)
			B += q2.x * q2.y;  // Σ(xi*yi)
			C += q2.x;         // Σ(xi)
			D += q2.x * q2.z;  // Σ(xi*zi)
			E += q2.y * q2.y;  // Σ(yi^2)
			F += q2.y;         // Σ(yi)
			G += q2.y * q2.z;  // Σ(yi*zi)
			H += q2.z;
		}

		// 行列と逆行列を作り、逆行列の一番右の列を取り出す
		Matrix4x4 matrix = new Matrix4x4 (
			new Vector4 (A, B, C, 0),
			new Vector4 (B, E, F, 0),
			new Vector4 (C, F, n, 0),
			new Vector4 (-D, -G, -H, 1));
		Vector3 solution = matrix.inverse.GetColumn(3);
		float a = solution.x;
		float b = solution.y;
		float c = solution.z;

		// 重心と平面の法線を作り、元の座標系に変換する
		Quaternion inverseRotation = Quaternion.Inverse (rotation);
		Vector3 planePoint = inverseRotation * new Vector3 (C/n, F/n, H/n);
		Vector3 planeNormal = inverseRotation * new Vector3(a, b, -1);

		// 標準偏差で平面の広さを決める
		float variance = inliers.Average (q => Vector3.SqrMagnitude (q - planePoint));
		float sd = Mathf.Sqrt (variance);

		// 平面のプレハブをインスタンス化
		GameObject plane = Instantiate (planePrefab, planePoint, Quaternion.LookRotation (planeNormal));
		plane.transform.localScale = new Vector3 (sd, sd, 1);
		Destroy (plane, 0.25f);
	}
}
