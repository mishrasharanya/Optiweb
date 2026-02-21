class UploadModal {
  constructor(options = {}) {
    this.acceptedTypes = options.acceptedTypes || [".csv"];
    this.onFileParsed = options.onFileParsed || function () {};
    this.modal = null;
    this.init();
  }

  init() {
    this.createModal();
    this.attachEvents();
  }

  createModal() {
    this.modal = document.createElement("div");
    this.modal.className = "upload-modal hidden";

    this.modal.innerHTML = `
      <div class="upload-modal-overlay"></div>
      <div class="upload-modal-content">
        <span class="upload-close">&times;</span>
        <h2>Upload File</h2>
        <div class="upload-drop-zone">
          Drag & Drop your file here<br>or click to browse
          <input type="file" class="upload-input" hidden />
        </div>
      </div>
    `;

    document.body.appendChild(this.modal);

    this.overlay = this.modal.querySelector(".upload-modal-overlay");
    this.content = this.modal.querySelector(".upload-modal-content");
    this.closeBtn = this.modal.querySelector(".upload-close");
    this.dropZone = this.modal.querySelector(".upload-drop-zone");
    this.fileInput = this.modal.querySelector(".upload-input");

    this.fileInput.setAttribute(
      "accept",
      this.acceptedTypes.join(",")
    );
  }

  attachEvents() {
    this.closeBtn.addEventListener("click", () => this.close());
    this.overlay.addEventListener("click", () => this.close());

    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape") this.close();
    });

    this.dropZone.addEventListener("click", () => {
      this.fileInput.click();
    });

    this.fileInput.addEventListener("change", (e) => {
      this.handleFile(e.target.files[0]);
    });

    ["dragenter", "dragover", "dragleave", "drop"].forEach(event => {
      this.dropZone.addEventListener(event, e => {
        e.preventDefault();
        e.stopPropagation();
      });
    });

    this.dropZone.addEventListener("dragover", () => {
      this.dropZone.classList.add("dragover");
    });

    this.dropZone.addEventListener("dragleave", () => {
      this.dropZone.classList.remove("dragover");
    });

    this.dropZone.addEventListener("drop", (e) => {
      this.dropZone.classList.remove("dragover");
      const file = e.dataTransfer.files[0];
      this.handleFile(file);
    });
  }

  handleFile(file) {
    if (!file) return;

    const isValid = this.acceptedTypes.some(type =>
      file.name.endsWith(type.replace(".", ""))
    );

    if (!isValid) {
      alert("Invalid file type.");
      return;
    }

    const reader = new FileReader();

    reader.onload = (event) => {
      const result = event.target.result;

      // Basic CSV parsing
      const parsed = result
        .split("\n")
        .map(row => row.split(",").map(cell => cell.trim()));

      this.onFileParsed(this.currentStep, parsed);
      this.close();
    };

    reader.readAsText(file);
  }

  open(step = null) {
    this.currentStep = step;
    this.modal.classList.remove("hidden");
    document.body.style.overflow = "hidden";
  }

  close() {
    this.modal.classList.add("hidden");
    document.body.style.overflow = "";
    this.fileInput.value = "";
  }

  destroy() {
    this.modal.remove();
  }
}