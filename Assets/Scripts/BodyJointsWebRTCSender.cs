using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Net;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.Networking;
using Meta.XR.Movement.Retargeting;
using UnityEngine.XR;
using Newtonsoft.Json;

public class BodyJointsWebRTCSender : MonoBehaviour
{
    [Tooltip("Base HTTP URL of the signaling server.")]
    public string signalingServerBaseUrl;

    [Tooltip("Frames per second to send; <= 0 sends every frame (fastest)")]
    public int sendFps = 0;

	

    [Tooltip("Automatically start sending on Start()")]
    public bool autoStart = true;

	[Tooltip("Enable additional verbose debug logging for WebRTC and networking")]
	public bool verboseLogging = true;

	[Header("Data Sources")]
	[Tooltip("Assign the CharacterRetargeter that drives the avatar. If not set, it will auto-find in the scene.")]
	public CharacterRetargeter retargeter;

	[Tooltip("If true, send local joint positions relative to parent; otherwise world positions.")]
	public bool retargeterUseLocalSpace = false;

	[Tooltip("Include retargeted joints section in JSON payload")] 
	public bool includeRetargetedJoints = true;

	[Tooltip("Include controller status section in JSON payload")] 
	public bool includeControllerStatus = true;

	[Tooltip("Include camera rig transform in JSON payload")] 
	public bool includeCameraRig = true;



    private RTCPeerConnection _peerConnection;
    private RTCDataChannel _dataChannel;
    private Coroutine _runCoroutine;
    private bool _isRunning;

    private void Start()
    {
        if (autoStart)
        {
            StartSending();
        }
    }

    public void StartSending()
    {
        if (_isRunning)
        {
            return;
        }
        if (sendFps <= 0)
        {
            Debug.Log("[WebRTC] sendFps <= 0 -> using fastest rate (every frame)");
        }
		Debug.Log($"[WebRTC] StartSending -> server={signalingServerBaseUrl}, fps={sendFps}");
		Debug.Log($"[WebRTC] Platform={Application.platform}, Reachability={Application.internetReachability}");
        _runCoroutine = StartCoroutine(RunClientCoroutine());
        _isRunning = true;
    }

    public void StopSending()
    {
        if (!_isRunning)
        {
            return;
        }

        if (_runCoroutine != null)
        {
            StopCoroutine(_runCoroutine);
            _runCoroutine = null;
        }

        CleanupPeerConnection();
        _isRunning = false;
    }

    private void OnDestroy()
    {
        StopSending();
    }

    private IEnumerator RunClientCoroutine()
    {
		// Parse and log URL/DNS info early for easier debugging
		if (Uri.TryCreate(signalingServerBaseUrl, UriKind.Absolute, out var uri))
		{
			Debug.Log($"[WebRTC] Signaling URL parsed: scheme={uri.Scheme}, host={uri.Host}, port={(uri.IsDefaultPort ? -1 : uri.Port)}");
			if (uri.Scheme == "http" && Application.platform == RuntimePlatform.Android)
			{
				Debug.LogWarning("[WebRTC] Using HTTP on Android; ensure cleartext traffic is allowed via Network Security Config and manifest.");
			}
			try
			{
				var addrs = Dns.GetHostAddresses(uri.Host);
				var firstAddrs = addrs.Select(a => a.ToString()).Take(3).ToArray();
				Debug.Log($"[WebRTC] DNS {uri.Host} -> [{string.Join(", ", firstAddrs)}]{(addrs.Length > 3 ? ", ..." : string.Empty)}");
			}
			catch (Exception ex)
			{
				Debug.LogWarning($"[WebRTC] DNS resolution failed for {uri.Host}: {ex.Message}");
			}
		}
		else
		{
			Debug.LogError("[WebRTC] Invalid signalingServerBaseUrl: " + signalingServerBaseUrl);
		}

        // Create PeerConnection with a public STUN server
        var config = GetDefaultRTCConfiguration();
        _peerConnection = new RTCPeerConnection(ref config);
        Debug.Log("[WebRTC] Created RTCPeerConnection");

        bool stopRequested = false;
        _peerConnection.OnConnectionStateChange = state =>
        {
            Debug.Log($"[WebRTC] PC state: {state}");
            if (state == RTCPeerConnectionState.Failed || state == RTCPeerConnectionState.Disconnected || state == RTCPeerConnectionState.Closed)
            {
                stopRequested = true;
            }
        };

		_peerConnection.OnIceConnectionChange = state =>
		{
			Debug.Log($"[WebRTC] ICE connection state: {state}");
		};

		_peerConnection.OnIceGatheringStateChange = state =>
		{
			Debug.Log($"[WebRTC] ICE gathering state: {state}");
		};

		// Note: Some Unity.WebRTC versions do not expose a signaling state change callback.
		// We will log signaling state at key steps instead of subscribing to an event.

		int iceCandidateCount = 0;
		_peerConnection.OnIceCandidate = candidate =>
		{
			iceCandidateCount++;
			if (verboseLogging)
			{
				Debug.Log($"[WebRTC] ICE candidate #{iceCandidateCount} mid={candidate.SdpMid} mline={candidate.SdpMLineIndex} len={(candidate.Candidate?.Length ?? 0)}");
			}
			else
			{
				Debug.Log($"[WebRTC] ICE candidate #{iceCandidateCount}");
			}
		};

		_peerConnection.OnNegotiationNeeded = () =>
		{
			Debug.Log("[WebRTC] OnNegotiationNeeded");
		};

        // Create data channel before offer
        _dataChannel = _peerConnection.CreateDataChannel("joints");

        bool channelOpen = false;
        _dataChannel.OnOpen = () =>
        {
			Debug.Log($"[WebRTC] Data channel '{_dataChannel.Label}' open (ordered={_dataChannel.Ordered}, negotiated={_dataChannel.Negotiated}, id={_dataChannel.Id})");
            channelOpen = true;
        };
        _dataChannel.OnClose = () =>
        {
            Debug.Log($"[WebRTC] Data channel '{_dataChannel.Label}' closed");
            stopRequested = true;
        };
		_dataChannel.OnMessage = bytes =>
		{
			Debug.Log($"[WebRTC] Data channel message received (len={bytes?.Length ?? 0})");
		};

		// Create offer
        Debug.Log("[WebRTC] Creating SDP offer...");
        var offerOp = _peerConnection.CreateOffer();
        yield return offerOp;
        if (offerOp.IsError)
        {
            Debug.LogError($"[WebRTC] CreateOffer error: {offerOp.Error.message}");
            yield break;
        }
        var offerDesc = offerOp.Desc;
		if (verboseLogging)
		{
			Debug.Log($"[WebRTC] Offer SDP length={offerDesc.sdp?.Length ?? 0}");
		}

        // Set local description
        var setLocalOp = _peerConnection.SetLocalDescription(ref offerDesc);
        yield return setLocalOp;
        if (setLocalOp.IsError)
        {
            Debug.LogError($"[WebRTC] SetLocalDescription error: {setLocalOp.Error.message}");
            yield break;
        }
		Debug.Log("[WebRTC] LocalDescription set");
		if (verboseLogging)
		{
			Debug.Log($"[WebRTC] Local SDP type={_peerConnection.LocalDescription.type}");
		}

        // Wait for ICE gathering to complete (or timeout) before sending the offer
        float iceWaitStart = Time.realtimeSinceStartup;
        const float iceWaitTimeout = 10f;
		Debug.Log($"[WebRTC] Waiting ICE gathering (<= {iceWaitTimeout}s), current={_peerConnection.GatheringState}");
        while (_peerConnection.GatheringState != RTCIceGatheringState.Complete && (Time.realtimeSinceStartup - iceWaitStart) < iceWaitTimeout)
        {
            yield return null;
        }
		Debug.Log($"[WebRTC] ICE gathering final state={_peerConnection.GatheringState}");
		if (iceCandidateCount == 0)
		{
			Debug.LogWarning("[WebRTC] No ICE candidates gathered. Network may block STUN/UDP.");
		}

		// Optional preflight connectivity check to the server root
		string baseUrl = signalingServerBaseUrl.TrimEnd('/') + "/";
		using (var preflight = UnityWebRequest.Get(baseUrl))
		{
			preflight.timeout = 5;
			Debug.Log($"[WebRTC] Preflight GET {baseUrl} (timeout={preflight.timeout}s)");
			UnityWebRequestAsyncOperation preflightOp = null;
			try
			{
			preflightOp = preflight.SendWebRequest();
			preflightOp.completed += _ =>
			{
				try
				{
					var res = preflight.result;
					var code = preflight.responseCode;
					var err = preflight.error;
					var bodyLen = preflight.downloadHandler != null ? (preflight.downloadHandler.text?.Length ?? 0) : 0;
					Debug.Log($"[WebRTC] Preflight completed: result={res}, code={code}, error={err}, len={bodyLen}");
				}
				catch (Exception ex2)
				{
					Debug.LogError($"[WebRTC] Preflight completion handler exception: {ex2.Message}");
				}
			};
			}
			catch (Exception ex)
			{
				Debug.LogError($"[WebRTC] Preflight SendWebRequest() exception: {ex.Message}");
				yield break;
			}
			yield return preflightOp;
			if (preflight.result != UnityWebRequest.Result.Success)
			{
				Debug.LogWarning($"[WebRTC] Preflight failed: code={preflight.responseCode}, error={preflight.error}, body={(preflight.downloadHandler != null ? preflight.downloadHandler.text : "<no body>")}");
			}
			else
			{
				Debug.Log($"[WebRTC] Preflight success: HTTP {preflight.responseCode}, len={(preflight.downloadHandler != null ? (preflight.downloadHandler.text?.Length ?? 0) : 0)}");
			}
		}

		// Post offer to signaling server and receive answer
        string typeString = _peerConnection.LocalDescription.type switch
        {
            RTCSdpType.Offer => "offer",
            RTCSdpType.Answer => "answer",
            RTCSdpType.Pranswer => "pranswer",
            RTCSdpType.Rollback => "rollback",
            _ => "offer"
        };
        string offerJson = $"{{\"sdp\":\"{EscapeJson(_peerConnection.LocalDescription.sdp)}\",\"type\":\"{typeString}\"}}";
        string offerUrl = signalingServerBaseUrl.TrimEnd('/') + "/offer";

		using (var request = new UnityWebRequest(offerUrl, UnityWebRequest.kHttpVerbPOST))
		{
			byte[] bodyRaw = Encoding.UTF8.GetBytes(offerJson);
			request.uploadHandler = new UploadHandlerRaw(bodyRaw);
			request.downloadHandler = new DownloadHandlerBuffer();
			request.SetRequestHeader("Content-Type", "application/json");
			request.timeout = 15;

			Debug.Log($"[WebRTC] POST {offerUrl} (sdpLength={_peerConnection.LocalDescription.sdp?.Length ?? 0})");
			if (verboseLogging)
			{
				Debug.Log($"[WebRTC] Request headers: Content-Type=application/json; timeout={request.timeout}s");
			}

			UnityWebRequestAsyncOperation postOp = null;
			try
			{
				postOp = request.SendWebRequest();
				postOp.completed += _ =>
				{
					try
					{
						var res = request.result;
						var code = request.responseCode;
						var err = request.error;
						var bodyLen = request.downloadHandler != null ? (request.downloadHandler.text?.Length ?? 0) : 0;
						Debug.Log($"[WebRTC] Offer POST completed: result={res}, code={code}, error={err}, len={bodyLen}");
					}
					catch (Exception ex2)
					{
						Debug.LogError($"[WebRTC] Offer POST completion handler exception: {ex2.Message}");
					}
				};
			}
			catch (Exception ex)
			{
				Debug.LogError($"[WebRTC] Offer POST SendWebRequest() exception: {ex.Message}");
				yield break;
			}
			yield return postOp;

			if (request.result != UnityWebRequest.Result.Success)
			{
				var respCode = request.responseCode;
				var errMsg = request.error;
				var respText = request.downloadHandler != null ? request.downloadHandler.text : "<no body>";
				Debug.LogError($"[WebRTC] Offer POST failed: {respCode} - {errMsg}\n{respText}");
				Debug.LogError($"[WebRTC] Reached URL: {offerUrl}. Check server is reachable from device, same LAN, and firewall/port 8080.");
				yield break;
			}

			// Expecting JSON { \"sdp\": \"...\", \"type\": \"answer\" }
			var answerText = request.downloadHandler != null ? request.downloadHandler.text : string.Empty;
			Debug.Log($"[WebRTC] Offer POST success HTTP {request.responseCode}; answerLen={answerText?.Length ?? 0}");
			if (!TryParseSdpAnswer(answerText, out var answerType, out var answerSdp))
			{
				Debug.LogError($"[WebRTC] Failed to parse SDP answer JSON: {answerText}");
				yield break;
			}

			var answerDesc = new RTCSessionDescription { type = answerType, sdp = answerSdp };
			var setRemoteOp = _peerConnection.SetRemoteDescription(ref answerDesc);
			yield return setRemoteOp;
			if (setRemoteOp.IsError)
			{
				Debug.LogError($"[WebRTC] SetRemoteDescription error: {setRemoteOp.Error.message}");
				yield break;
			}
			Debug.Log("[WebRTC] RemoteDescription applied");
			if (verboseLogging)
			{
				Debug.Log($"[WebRTC] Answer SDP length={answerDesc.sdp?.Length ?? 0}");
			}
		}

        // Wait for channel open (with timeout)
        float openStart = Time.realtimeSinceStartup;
        const float openTimeout = 15f;
		while (!channelOpen && (Time.realtimeSinceStartup - openStart) < openTimeout)
        {
            yield return null;
        }
        if (!channelOpen)
        {
			Debug.LogError("[WebRTC] Timed out waiting for data channel to open");
			Debug.LogError($"[WebRTC] ICE candidates observed: {iceCandidateCount}");
            yield break;
        }

        // Sender loop at fixed FPS (or per-frame if unthrottled)
        float interval = sendFps > 0 ? 1.0f / sendFps : 0f;
        if (sendFps > 0)
        {
            Debug.Log($"[WebRTC] Starting joints sender at {sendFps} FPS");
        }
        else
        {
            Debug.Log("[WebRTC] Starting joints sender at fastest rate (every frame)");
        }
        while (!stopRequested)
        {
            long tsMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            string payload = BuildStatusJson(tsMs);
            try
            {
				_dataChannel.Send(payload);
				if (verboseLogging)
				{
					Debug.Log($"[WebRTC] Sent joints payload: {payload}");
				}
            }
            catch (Exception ex)
            {
                Debug.LogError($"[WebRTC] Failed to send on data channel: {ex}");
                break;
            }

            if (sendFps > 0)
            {
                yield return new WaitForSecondsRealtime(interval);
            }
            else
            {
                // Unthrottled: send once per frame (fastest available)
                yield return null;
            }
        }

        Debug.Log("[WebRTC] Sender loop finished");
        CleanupPeerConnection();
    }

    private void CleanupPeerConnection()
    {
        try
        {
            if (_dataChannel != null)
            {
                _dataChannel.Close();
                _dataChannel.Dispose();
            }
        }
        catch (Exception)
        {
        }
        finally
        {
            _dataChannel = null;
        }

        try
        {
            if (_peerConnection != null)
            {
                _peerConnection.Close();
                _peerConnection.Dispose();
            }
        }
        catch (Exception)
        {
        }
        finally
        {
            _peerConnection = null;
        }
    }

    private static RTCConfiguration GetDefaultRTCConfiguration()
    {
        var config = new RTCConfiguration
        {
            iceServers = new[]
            {
                new RTCIceServer { urls = new[] { "stun:stun.l.google.com:19302" } }
            }
        };
        return config;
    }

	private string BuildStatusJson(long timestampMs)
	{
		var payload = new Dictionary<string, object>(3)
		{
			["timestamp_ms"] = timestampMs
		};

		if (includeRetargetedJoints)
		{
			var joints = BuildRetargetedJointsMap();
			if (joints != null && joints.Count > 0)
			{
				payload["retargeted_joints"] = joints;
			}
		}

		if (includeControllerStatus)
		{
			var controllers = BuildControllersMap();
			if (controllers != null)
			{
				payload["controllers"] = controllers;
			}
		}

		if (includeCameraRig)
		{
			var rig = BuildCameraRigMap();
			if (rig != null)
			{
				payload["camera_rig"] = rig;
			}
		}

		return JsonConvert.SerializeObject(payload);
	}

	private Dictionary<string, object> BuildRetargetedJointsMap()
	{
		try
		{
			if (retargeter == null)
			{
				retargeter = FindAnyObjectByType<CharacterRetargeter>();
			}
			if (retargeter == null || retargeter.JointPairs == null || retargeter.JointPairs.Length == 0)
			{
				return new Dictionary<string, object>(0);
			}

			var map = new Dictionary<string, object>(retargeter.JointPairs.Length);
			var pairs = retargeter.JointPairs;
			for (int i = 0; i < pairs.Length; i++)
			{
				var jointTransform = pairs[i].Joint;
				if (jointTransform == null) continue;
				var pos = retargeterUseLocalSpace ? jointTransform.localPosition : jointTransform.position;
				map[jointTransform.name] = new Dictionary<string, float>
				{
					["x"] = pos.x,
					["y"] = pos.y,
					["z"] = pos.z
				};
			}
			return map;
		}
		catch (Exception ex)
		{
			Debug.LogWarning($"[WebRTC] Failed to build retargeted joints map: {ex.Message}");
			return new Dictionary<string, object>(0);
		}
	}

	private Dictionary<string, object> BuildControllersMap()
	{
		var controllers = new Dictionary<string, object>(2)
		{
			["left"] = BuildControllerDictionary(XRNode.LeftHand),
			["right"] = BuildControllerDictionary(XRNode.RightHand)
		};
		return controllers;
	}

	private Dictionary<string, object> BuildControllerDictionary(XRNode node)
	{
		var device = InputDevices.GetDeviceAtXRNode(node);
		var obj = new Dictionary<string, object>(16)
		{
			["valid"] = device.isValid
		};
		if (!device.isValid) return obj;

		if (device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked))
		{
			obj["tracked"] = isTracked;
		}
		if (device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
		{
			obj["position"] = new Dictionary<string, float> { ["x"] = pos.x, ["y"] = pos.y, ["z"] = pos.z };
		}
		if (device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
		{
			obj["rotation_quat"] = new Dictionary<string, float> { ["x"] = rot.x, ["y"] = rot.y, ["z"] = rot.z, ["w"] = rot.w };
		}
		if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick))
		{
			var stickObj = new Dictionary<string, object> { ["x"] = stick.x, ["y"] = stick.y };
			if (device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool stickClick))
			{
				stickObj["click"] = stickClick;
			}
			if (device.TryGetFeatureValue(CommonUsages.primary2DAxisTouch, out bool stickTouch))
			{
				stickObj["touch"] = stickTouch;
			}
			obj["stick"] = stickObj;
		}
		if (device.TryGetFeatureValue(CommonUsages.trigger, out float trigger))
		{
			obj["trigger"] = trigger;
		}
		if (device.TryGetFeatureValue(CommonUsages.grip, out float grip))
		{
			obj["grip"] = grip;
		}
		if (device.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryBtn))
		{
			obj["primaryButton"] = primaryBtn;
		}
		if (device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryBtn))
		{
			obj["secondaryButton"] = secondaryBtn;
		}
		if (device.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerBtn))
		{
			obj["triggerButton"] = triggerBtn;
		}
		if (device.TryGetFeatureValue(CommonUsages.gripButton, out bool gripBtn))
		{
			obj["gripButton"] = gripBtn;
		}
		if (device.TryGetFeatureValue(CommonUsages.menuButton, out bool menuBtn))
		{
			obj["menuButton"] = menuBtn;
		}
		return obj;
	}

	private Dictionary<string, object> BuildCameraRigMap()
	{
		var obj = new Dictionary<string, object>(8);
		var head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
		if (!head.isValid)
		{
			obj["valid"] = false;
			return obj;
		}
		obj["valid"] = true;
		if (head.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked))
		{
			obj["tracked"] = isTracked;
		}
		if (head.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 dpos))
		{
			obj["position"] = new Dictionary<string, float> { ["x"] = dpos.x, ["y"] = dpos.y, ["z"] = dpos.z };
		}
		if (head.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion drot))
		{
			obj["rotation_quat"] = new Dictionary<string, float> { ["x"] = drot.x, ["y"] = drot.y, ["z"] = drot.z, ["w"] = drot.w };
		}
		return obj;
	}

	private bool TryAppendRetargetedJoints(StringBuilder sb)
	{
		try
		{
			if (retargeter == null)
			{
				retargeter = FindAnyObjectByType<CharacterRetargeter>();
			}
			if (retargeter == null || retargeter.JointPairs == null || retargeter.JointPairs.Length == 0)
			{
				return false;
			}

			sb.Append(',');
			sb.Append("\"retargeted_joints\":{");
			bool first = true;
			var pairs = retargeter.JointPairs;
			for (int i = 0; i < pairs.Length; i++)
			{
				var jointTransform = pairs[i].Joint;
				if (jointTransform == null) continue;
				var pos = retargeterUseLocalSpace ? jointTransform.localPosition : jointTransform.position;
				if (!first) sb.Append(',');
				first = false;
				AppendVec3(sb, jointTransform.name, pos.x, pos.y, pos.z);
			}
			sb.Append('}');
			return true;
		}
		catch (Exception ex)
		{
			Debug.LogWarning($"[WebRTC] Failed to append retargeted joints: {ex.Message}");
			return false;
		}
	}

	private bool TryAppendControllers(StringBuilder sb, bool hasExistingSection)
	{
		try
		{
			if (!hasExistingSection) { /* no comma needed before first section */ }
			sb.Append(',');
			sb.Append("\"controllers\":{");
			sb.Append("\"left\":");
			AppendControllerObject(sb, XRNode.LeftHand);
			sb.Append(',');
			sb.Append("\"right\":");
			AppendControllerObject(sb, XRNode.RightHand);
			sb.Append('}');
			return true;
		}
		catch (Exception ex)
		{
			Debug.LogWarning($"[WebRTC] Failed to append controllers: {ex.Message}");
			return false;
		}
	}

	private void AppendControllerObject(StringBuilder sb, XRNode node)
	{
		var device = InputDevices.GetDeviceAtXRNode(node);
		sb.Append('{');
		bool first = true;

		void AppendBool(string name, bool value)
		{
			if (!first) sb.Append(','); first = false;
			sb.Append('"'); sb.Append(name); sb.Append("\":"); sb.Append(value ? "true" : "false");
		}

		void AppendFloat(string name, float value)
		{
			if (!first) sb.Append(','); first = false;
			sb.Append('"'); sb.Append(name); sb.Append("\":"); sb.Append(value.ToString("F6"));
		}

		void AppendVec3Inline(string name, Vector3 v)
		{
			if (!first) sb.Append(','); first = false;
			sb.Append('"'); sb.Append(name); sb.Append("\":{");
			sb.Append("\"x\":"); sb.Append(v.x.ToString("F6")); sb.Append(',');
			sb.Append("\"y\":"); sb.Append(v.y.ToString("F6")); sb.Append(',');
			sb.Append("\"z\":"); sb.Append(v.z.ToString("F6"));
			sb.Append('}');
		}

		void AppendQuatInline(string name, Quaternion q)
		{
			if (!first) sb.Append(','); first = false;
			sb.Append('"'); sb.Append(name); sb.Append("\":{");
			sb.Append("\"x\":"); sb.Append(q.x.ToString("F6")); sb.Append(',');
			sb.Append("\"y\":"); sb.Append(q.y.ToString("F6")); sb.Append(',');
			sb.Append("\"z\":"); sb.Append(q.z.ToString("F6")); sb.Append(',');
			sb.Append("\"w\":"); sb.Append(q.w.ToString("F6"));
			sb.Append('}');
		}

		if (!device.isValid)
		{
			AppendBool("valid", false);
			sb.Append('}');
			return;
		}

		AppendBool("valid", true);

		if (device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked))
		{
			AppendBool("tracked", isTracked);
		}
		if (device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
		{
			AppendVec3Inline("position", pos);
		}
		if (device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
		{
			AppendQuatInline("rotation_quat", rot);
		}
		if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick))
		{
			if (!first) sb.Append(','); first = false;
			sb.Append("\"stick\":{");
			sb.Append("\"x\":"); sb.Append(stick.x.ToString("F6")); sb.Append(',');
			sb.Append("\"y\":"); sb.Append(stick.y.ToString("F6"));
			if (device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool stickClick))
			{
				sb.Append(','); sb.Append("\"click\":"); sb.Append(stickClick ? "true" : "false");
			}
			if (device.TryGetFeatureValue(CommonUsages.primary2DAxisTouch, out bool stickTouch))
			{
				sb.Append(','); sb.Append("\"touch\":"); sb.Append(stickTouch ? "true" : "false");
			}
			sb.Append('}');
		}
		if (device.TryGetFeatureValue(CommonUsages.trigger, out float trigger))
		{
			AppendFloat("trigger", trigger);
		}
		if (device.TryGetFeatureValue(CommonUsages.grip, out float grip))
		{
			AppendFloat("grip", grip);
		}
		if (device.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryBtn))
		{
			AppendBool("primaryButton", primaryBtn);
		}
		if (device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryBtn))
		{
			AppendBool("secondaryButton", secondaryBtn);
		}
		if (device.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerBtn))
		{
			AppendBool("triggerButton", triggerBtn);
		}
		if (device.TryGetFeatureValue(CommonUsages.gripButton, out bool gripBtn))
		{
			AppendBool("gripButton", gripBtn);
		}
		if (device.TryGetFeatureValue(CommonUsages.menuButton, out bool menuBtn))
		{
			AppendBool("menuButton", menuBtn);
		}

		sb.Append('}');
	}

    private static string EscapeJson(string s)
    {
        if (string.IsNullOrEmpty(s)) return string.Empty;
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n").Replace("\r", "\\r");
    }

    private static bool TryParseSdpAnswer(string json, out RTCSdpType type, out string sdp)
    {
        // Minimal JSON parsing without external libs; looks for keys "type" and "sdp"
        type = RTCSdpType.Answer;
        sdp = string.Empty;
        try
        {
            // Very basic and assumes flat JSON with double-quoted keys
            string typeValue = ExtractJsonString(json, "type");
            string sdpValue = ExtractJsonString(json, "sdp");
            if (string.IsNullOrEmpty(typeValue) || string.IsNullOrEmpty(sdpValue))
            {
                return false;
            }

            // Unity.WebRTC expects lowercase enum names
            if (string.Equals(typeValue, "answer", StringComparison.OrdinalIgnoreCase))
            {
                type = RTCSdpType.Answer;
            }
            else if (string.Equals(typeValue, "offer", StringComparison.OrdinalIgnoreCase))
            {
                type = RTCSdpType.Offer;
            }
            else if (string.Equals(typeValue, "pranswer", StringComparison.OrdinalIgnoreCase))
            {
                type = RTCSdpType.Pranswer;
            }
            else
            {
                return false;
            }

            sdp = sdpValue;
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[WebRTC] JSON parse error: {ex.Message}");
            return false;
        }
    }

    private static string ExtractJsonString(string json, string key)
    
    {
        // Find "key":"value" and return unescaped value (handles basic escapes for \" and \\ and \n/\r)
        int keyIndex = json.IndexOf("\"" + key + "\"", StringComparison.Ordinal);
        if (keyIndex < 0) return string.Empty;
        int colon = json.IndexOf(':', keyIndex);
        if (colon < 0) return string.Empty;
        int firstQuote = json.IndexOf('"', colon + 1);
        if (firstQuote < 0) return string.Empty;
        int i = firstQuote + 1;
        var sb = new StringBuilder();
        while (i < json.Length)
        {
            char c = json[i++];
            if (c == '\\')
            {
                if (i >= json.Length) break;
                char esc = json[i++];
                switch (esc)
                {
                    case '"': sb.Append('"'); break;
                    case '\\': sb.Append('\\'); break;
                    case 'n': sb.Append('\n'); break;
                    case 'r': sb.Append('\r'); break;
                    case 't': sb.Append('\t'); break;
                    default: sb.Append(esc); break;
                }
            }
            else if (c == '"')
            {
                break;
            }
            else
            {
                sb.Append(c);
            }
        }
        return sb.ToString();
    }

    private static string GenerateDummyJointsJson(long timestampMs, float noise)
    {
        float t = Time.realtimeSinceStartup;
        float headY = 1.6f + 0.02f * Mathf.Sin(t);
        float leftX = 0.2f + 0.05f * Mathf.Sin(0.5f * t + 0.3f);
        float rightX = -0.2f + 0.05f * Mathf.Cos(0.4f * t - 0.2f);
        Func<float> jitter = () => (UnityEngine.Random.value - 0.5f) * (2.0f * Mathf.Max(0.0f, noise));

        // Build JSON string manually to avoid dependency on external JSON libs
        var sb = new StringBuilder(256);
        sb.Append('{');
        sb.Append("\"timestamp_ms\":"); sb.Append(timestampMs);
        sb.Append(",\"joints\":{");
        AppendVec3(sb, "Head", 0.0f + jitter(), headY, 0.0f + jitter()); sb.Append(',');
        AppendVec3(sb, "LeftHand", leftX, 1.2f + jitter(), 0.3f + jitter()); sb.Append(',');
        AppendVec3(sb, "RightHand", rightX, 1.2f + jitter(), 0.3f + jitter()); sb.Append(',');
        AppendVec3(sb, "Hips", 0.0f + jitter(), 1.0f + jitter(), 0.0f + jitter());
        sb.Append('}');
        sb.Append('}');
        return sb.ToString();
    }

    private static void AppendVec3(StringBuilder sb, string name, float x, float y, float z)
    {
        sb.Append('"'); sb.Append(name); sb.Append('"'); sb.Append(':');
        sb.Append('{');
        sb.Append("\"x\":"); sb.Append(x.ToString("F6")); sb.Append(',');
        sb.Append("\"y\":"); sb.Append(y.ToString("F6")); sb.Append(',');
        sb.Append("\"z\":"); sb.Append(z.ToString("F6"));
        sb.Append('}');
    }
}
