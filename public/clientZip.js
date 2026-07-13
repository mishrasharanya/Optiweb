(function () {
  async function downloadNeosZip({
    jobNumber,
    dat,
    mod,
    run,
    neosOutput,
    jobOut,
    emailStatus,
    summary
  }) {
    if (!window.JSZip) {
      throw new Error("JSZip is not loaded in the browser.");
    }

    const zip = new window.JSZip();

    zip.file("model.mod", mod || "");
    zip.file("data.dat", dat || "");
    zip.file("run.run", run || "");
    zip.file("solver_output.txt", neosOutput || "");
    zip.file("job.out", jobOut || "");
    zip.file("email_status.txt", emailStatus || "");
    zip.file("summary.txt", summary || "");

    const blob = await zip.generateAsync({ type: "blob" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");

    link.href = url;
    link.download = `optiweb-job-${jobNumber || "results"}.zip`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
  }

  window.OptiWebZip = { downloadNeosZip };
}());
