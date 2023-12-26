// import { kendo } from "./demo/kendo";
import { kendo20231215 } from "../demo/kendo20231215";
import { init } from "../initGlobal";


init();
export function setupGanja() {
  document
    .getElementById("ganja-container")
    .appendChild(document.createElement("canvas"));

  kendo20231215();
}