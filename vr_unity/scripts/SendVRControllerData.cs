// ???????????????????????????????????
[SerializeField] private GameObject leftController;
[SerializeField] private InputActionProperty leftTrackingButton;
[SerializeField] private InputActionProperty leftGripperButton;
private bool leftTrackingButtonTriggered = false;
private bool leftGripperButtonTriggered = false;

// ?? OnEnable: ???????????????????????????????
void OnEnable()
{
    rightTrackingButton.action.performed += ctx => rightTrackingButtonTriggered = true;
    rightGripperButton.action.performed += ctx => rightGripperButtonTriggered = true;
    leftTrackingButton.action.performed += ctx => leftTrackingButtonTriggered = true;
    leftGripperButton.action.performed += ctx => leftGripperButtonTriggered = true;
}

// ?? OnDisable: ????????????????????????????????
void OnDisable()
{
    rightTrackingButton.action.performed -= ctx => rightTrackingButtonTriggered = true;
    rightGripperButton.action.performed -= ctx => rightGripperButtonTriggered = true;
    leftTrackingButton.action.performed -= ctx => leftTrackingButtonTriggered = true;
    leftGripperButton.action.performed -= ctx => leftGripperButtonTriggered = true;
    stream?.Close();
    client?.Close();
    server?.Stop();
}

// ?? Update: ?????????????????????????????????????
void Update()
{
    if (client == null || !client.Connected) { /* ?????????????????? */ }
    if (Time.time - lastUpdateTime < targetInterval) return;
    lastUpdateTime = Time.time;

    // ????????????
    Vector3 rightPosition = rightController.transform.position;
    Quaternion rightRotation = rightController.transform.rotation;
    rightPosition = new Vector3(rightPosition.z, -rightPosition.x, rightPosition.y);
    rightRotation = new Quaternion(rightRotation.z, -rightRotation.x, rightRotation.y, -rightRotation.w);
    bool rightButton1 = rightTrackingButton.action.IsPressed();
    bool rightButton2 = rightGripperButton.action.IsPressed();

    // ?????????????
    Vector3 leftPosition = leftController.transform.position;
    Quaternion leftRotation = leftController.transform.rotation;
    leftPosition = new Vector3(leftPosition.z, -leftPosition.x, leftPosition.y);
    leftRotation = new Quaternion(leftRotation.z, -leftRotation.x, leftRotation.y, -leftRotation.w);
    bool leftButton1 = leftTrackingButton.action.IsPressed();
    bool leftButton2 = leftGripperButton.action.IsPressed();

    // ??????????????????
    string FormatFloat(float value) => value.ToString("F3");
    string data = $"{FormatFloat(rightPosition.x)},{FormatFloat(rightPosition.y)},{FormatFloat(rightPosition.z)}," +
                  $"{FormatFloat(rightRotation.x)},{FormatFloat(rightRotation.y)},{FormatFloat(rightRotation.z)},{FormatFloat(rightRotation.w)}," +
                  $"{(rightButton1 ? 1 : 0)},{(rightButton2 ? 1 : 0)}," +
                  $"{FormatFloat(leftPosition.x)},{FormatFloat(leftPosition.y)},{FormatFloat(leftPosition.z)}," +
                  $"{FormatFloat(leftRotation.x)},{FormatFloat(leftRotation.y)},{FormatFloat(leftRotation.z)},{FormatFloat(leftRotation.w)}," +
                  $"{(leftButton1 ? 1 : 0)},{(leftButton2 ? 1 : 0)}\n";

    // ?????????
    try
    {
        byte[] message = Encoding.UTF8.GetBytes(data);
        stream.Write(message, 0, message.Length);
    }
    catch (Exception e)
    {
        Debug.Log($"Error writing to stream: {e.Message}");
        client?.Close();
        stream = null;
        WaitForClientConnection();
    }

    rightTrackingButtonTriggered = false;
    rightGripperButtonTriggered = false;
    leftTrackingButtonTriggered = false;
    leftGripperButtonTriggered = false;
}