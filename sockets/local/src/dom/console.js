import {init} from "../initGlobal";


init();
//MAX_LINE_COUNT
export function setupConsole() {
  const c = document.getElementById("console");
  const log = function (line) {
    const hhmmss = new Date().toLocaleTimeString("zh");
    c.innerText = `${c.innerText}[${hhmmss}] ${line}\n`;
    c.scrollBy(0, 1000);
  }
  log("Console ready.");
  window.api.log = log;
  console.log("log")
}
