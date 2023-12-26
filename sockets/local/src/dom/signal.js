import { init } from "../initGlobal"


init();
window.state.signal = {};
export function hasTriggered(name) {
  if (window.state.signal[name] === true) {
    window.api.log(`Trigger ${name}.`);
    window.state.signal[name] = false;
    return true;
  }
  return false;
}
function displayStatus(status) {
  document.getElementById("status").innerText = status;
  window.api.log(`Set status to ${status}`);
}
function getStatus() {
  return document.getElementById("status").innerText;
}

export function setupSignal() {
  document.getElementById("signal-track").addEventListener("click", () => {
    window.state.signal.track = true;
    displayStatus("Track");
  });
  document.getElementById("signal-head").addEventListener("click", () => {
    window.state.signal.head = true;
    displayStatus("Head");
  });
  document.getElementById("signal-throat").addEventListener("click", () => {
    window.state.signal.throat = true;
    displayStatus("Throat");
  });
  document.getElementById("signal-hand").addEventListener("click", () => {
    window.state.signal.hand = true;
    displayStatus("Hand");
  });
  document.getElementById("signal-body").addEventListener("click", () => {
    window.state.signal.body = true;
    displayStatus("Body");
  });
  window.api.hasTriggered = hasTriggered;
  window.api.getStatus = getStatus;
}
