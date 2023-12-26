import { EditorView, basicSetup } from "codemirror"
import { javascript } from "@codemirror/lang-javascript"

import { kendo20231215 } from "../demo/kendo20231215";
import { replaceCanvas } from "../canvas";


// Content of the function as string.
function getFunctionContent(f) {
  const body = f.toString();
  return body.substring(
    body.indexOf("{") + 1, body.lastIndexOf("}")
  );
}

export function setupCodemirror() {
  // Create editor.
  let editor = new EditorView({
    extensions: [basicSetup, javascript()],
    parent: document.getElementById("codemirror-container")
  })
  
  // Manually set the content of the editor. 
  editor.contentDOM.innerText = getFunctionContent(kendo20231215);
  document.getElementById("right").scrollTo(0, 0);

  // Set event handlers.
  document.getElementById("update").addEventListener("click", () => {
    const t = editor.state.doc.toString();
    // Please do not attack yourself with this, thank you.
    eval(t);
  })
  document.getElementById("clear").addEventListener("click", () => {
    replaceCanvas(document.createElement("canvas"));
  })
}
