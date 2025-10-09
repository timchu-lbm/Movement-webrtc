using System;
using UnityEngine;
using TMPro;

public class IpInputField : MonoBehaviour
{
	[SerializeField]
	private TMP_InputField inputField;

	[SerializeField]
	private BodyJointsWebRTCSender sender;

	private void OnEnable()
	{
		if (inputField != null)
		{
			inputField.onEndEdit.AddListener(OnFieldEndEdit);
		}
	}

	private void OnDisable()
	{
		if (inputField != null)
		{
			inputField.onEndEdit.RemoveListener(OnFieldEndEdit);
		}
	}

	private void Start()
	{
		if (sender == null)
		{
			Debug.LogError("[IP Field] BodyJointsWebRTCSender not found or not assigned.");
			return;
		}
		if (inputField == null)
		{
			Debug.LogError("[IP Field] TMP_InputField not found or not assigned.");
			return;
		}

		var normalized = NormalizeServerUrl(inputField.text ?? string.Empty);
		if (normalized.Length == 0)
		{
			Debug.LogWarning("[IP Field] Input is empty; skipping auto-connect.");
			return;
		}

		if (!string.Equals(inputField.text, normalized, StringComparison.Ordinal))
		{
			inputField.text = normalized;
		}
		sender.signalingServerBaseUrl = normalized;
		TryReconnect();
	}

	private void OnFieldEndEdit(string _)
	{
		if (sender == null || inputField == null)
		{
			return;
		}
		var normalized = NormalizeServerUrl(inputField.text ?? string.Empty);
		if (normalized.Length == 0)
		{
			return;
		}
		if (!string.Equals(inputField.text, normalized, StringComparison.Ordinal))
		{
			inputField.text = normalized;
		}
		if (!string.Equals(sender.signalingServerBaseUrl, normalized, StringComparison.Ordinal))
		{
			sender.signalingServerBaseUrl = normalized;
			TryReconnect();
		}
	}

	private void TryReconnect()
	{
		try
		{
			sender.StopSending();
			sender.StartSending();
			Debug.Log($"[IP Field] Connecting to {sender.signalingServerBaseUrl}");
		}
		catch (Exception ex)
		{
			Debug.LogError($"[IP Field] Failed to (re)connect: {ex.Message}");
		}
	}

	private static string NormalizeServerUrl(string input)
	{
		if (string.IsNullOrWhiteSpace(input))
		{
			return string.Empty;
		}
		string s = input.Trim();
		if (s.EndsWith("/offer", StringComparison.OrdinalIgnoreCase))
		{
			s = s.Substring(0, s.Length - "/offer".Length).TrimEnd('/');
		}
		// Ensure scheme
		if (!s.StartsWith("http://", StringComparison.OrdinalIgnoreCase) &&
			!s.StartsWith("https://", StringComparison.OrdinalIgnoreCase))
		{
			s = "http://" + s;
		}
		// Append :8080 if no port present (simplified, IPv6 not handled)
		int schemeIdx = s.IndexOf("://", StringComparison.Ordinal);
		int authStart = schemeIdx >= 0 ? schemeIdx + 3 : 0;
		int authEnd = s.Length;
		int slashIdx = s.IndexOf('/', authStart);
		if (slashIdx >= 0) authEnd = slashIdx;
		int qIdx = s.IndexOf('?', authStart);
		if (qIdx >= 0 && qIdx < authEnd) authEnd = qIdx;
		int hashIdx = s.IndexOf('#', authStart);
		if (hashIdx >= 0 && hashIdx < authEnd) authEnd = hashIdx;
		string authority = s.Substring(authStart, authEnd - authStart);
		bool hasPort = authority.Contains(":");
		if (!hasPort)
		{
			s = s.Insert(authEnd, ":8080");
		}
		return s;
	}
}


