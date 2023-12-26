import { init } from "../initGlobal";


init();
window.state.shouldSendWebSocket = false;
let ws = null;
let sendWebsocketCounter = 0; // For use with `throttle`.
/**
 * @param {number} throttle Send every N calls. Set to `1` to send every call.
 */
export function sendWebSocket(message, throttle) {
  if (window.state.shouldSendWebSocket === false) {
    return;
  }
  // Throttle.
  if (throttle == null) {
    throttle = 1;
  }
  sendWebsocketCounter++;
  if (sendWebsocketCounter % throttle !== 0) {
    return;
  }
  // Check if WebSocket is open.
  // https://stackoverflow.com/a/54061045
  if (ws == null || ws.readyState !== ws.OPEN) {
    console.log("Attempting send, ws is not open.");
    window.state.shouldSendWebSocket = false;
    document.getElementById("send").checked = false;
    document.getElementById("listen").checked = false;
    return;
  }
  // Send.
  ws.send(message);
}

let shouldOpen = false;
let preventSleepTimeout = null;
const SLEEP_TIMEOUT_MS = 500;
// Close the WebSocket client.
function _close() {
  console.log("Close ws...");
  if (ws != null) {
    ws.close();
    ws = null;
  }
  shouldOpen = false;
  clearTimeout(preventSleepTimeout);
}

// Open the WebSocket client.
function _connect() {
  // Create WebSocket client.
  console.log("Open ws...");
  ws = new WebSocket("ws://localhost:9999");

  // Event listeners.
  ws.addEventListener("open", (_) => {
    console.log("Connected!");
  });
  ws.addEventListener("close", (_) => {
    if (shouldOpen) {
      console.log("Closed unexpectedly! reconnecting...");
      _connect();
    } else {
      _close();
    }
  });
  ws.addEventListener("message", (event) => {
    // console.log("Message from server ", event.data);
    console.log("Message from server");
    const data = JSON.parse(event.data);
    window.state.score = data.score;
    window.state.pose = data.pose;
    clearTimeout(preventSleepTimeout);
    preventSleepTimeout = setTimeout(() => {
      console.log("No message Timeout! reconnecting...");
      _connect();
    }, SLEEP_TIMEOUT_MS)
  });
}

export function setupWebSocket() {
  // Set event handlers for the "Listen" checkbox.
  document.getElementById("listen").addEventListener("change", (e) => {
    shouldOpen = e.target.checked;
    if (shouldOpen) { // On check.
      _connect();
    } else { // On uncheck.
      _close();
    }
  })
  // Set event handlers for the "Send" checkbox.
  document.getElementById("send").addEventListener("change", (e) => {
    window.state.shouldSendWebSocket = e.target.checked;
    console.log(`Set window.state.shouldSendWebSocket to ${window.state.shouldSendWebSocket}`);
  })
  window.api.sendWebSocket = sendWebSocket;
}