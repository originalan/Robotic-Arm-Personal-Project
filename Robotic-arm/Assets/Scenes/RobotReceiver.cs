using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;

public class RobotReceiver : MonoBehaviour
{
    Thread receiveThread;
    UdpClient client;
    public int port = 5005;

    public Transform baseJoint, shoulderJoint, elbowJoint;
    public Transform goalDot;

    string lastMessage = "";

    void Start() {
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    private void ReceiveData() {
        client = new UdpClient(port);
        while (true) {
            try {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = client.Receive(ref anyIP);
                lastMessage = Encoding.UTF8.GetString(data);
            } catch { }
        }
    }

    void Update() {
        if (!string.IsNullOrEmpty(lastMessage)) {
            string[] angles = lastMessage.Split(',');
            if (angles.Length >= 7) {
                // expect: [Base, Shoulder, Elbow, TargetX, TargetY, TargetZ, reachable?]
                float b = float.Parse(angles[0]);
                float s = float.Parse(angles[1]);
                float e = float.Parse(angles[2]);

                // Apply rotations (You may need to swap X, Y, or Z depending on your model)
                // We use 'Lerp' for that smooth feeling
                baseJoint.localRotation = Quaternion.Lerp(baseJoint.localRotation, Quaternion.Euler(0, b, 0), Time.deltaTime * 5f);
                shoulderJoint.localRotation = Quaternion.Lerp(shoulderJoint.localRotation, Quaternion.Euler(0, 0, s - 90), Time.deltaTime * 5f);
                elbowJoint.localRotation = Quaternion.Lerp(elbowJoint.localRotation, Quaternion.Euler(0, 0, e - 45), Time.deltaTime * 5f);

                float tx = float.Parse(angles[3]);
                float ty = float.Parse(angles[4]);
                float tz = float.Parse(angles[5]);

                // Unity uses (X, Y, Z) where Y is up. 
                goalDot.position = new Vector3(ty, tz, tx);

                int status = int.Parse(angles[6]);
                Renderer dotRender = goalDot.GetComponent<Renderer>();

                if (status == 1) {
                    dotRender.material.color = Color.red; // Unreachable
                } else {
                    dotRender.material.color = Color.green; // Reachable
                }
            }
        }
    }
}
