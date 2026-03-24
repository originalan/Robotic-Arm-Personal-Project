using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;

public class MouseToPython : MonoBehaviour
{
    public string IP = "127.0.0.1";
    public int port = 5006; // Use a DIFFERENT port than the receiver
    UdpClient client;

    void Start() {
        client = new UdpClient();
    }

    void Update() {
        if (Mouse.current.leftButton.isPressed) {
            Vector2 mousePos = Mouse.current.position.ReadValue();
            
            Ray ray = Camera.main.ScreenPointToRay(mousePos);
            RaycastHit hit;

            // Make sure your Floor has a 'Mesh Collider' or 'Box Collider'
            if (Physics.Raycast(ray, out hit)) {
                Vector3 p = hit.point;

                // Send to Python (Mapping: Unity Z -> Python X, Unity X -> Python Y)
                string message = $"{p.z},{p.x},{p.y}"; 
                
                byte[] data = Encoding.UTF8.GetBytes(message);
                client.Send(data, data.Length, IP, port);
            }
        }
    }

    private void OnApplicationQuit() {
        if (client != null) client.Close();
    }
}