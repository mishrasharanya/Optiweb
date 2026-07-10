const express = require("express");
const bodyParser = require("body-parser");
const NEOS = require("neos-js");
const JSZip = require("jszip");
const path = require("path");

const app = express();
const PORT = 3000;
const NEOS_XMLRPC_URL = "https://neos-server.org:3333";

console.log("RUNNING SERVER FILE:", __filename);

app.use(express.static(path.join(__dirname, "Public")));
app.use("/examples/neos", express.static(path.join(__dirname, "neos")));
app.use(
  "/vendor/three",
  express.static(path.join(__dirname, "node_modules", "three", "build"))
);
app.use(bodyParser.json({ limit: "100mb" }));

const jobs = new Map();

function isValidEmail(email) {
  return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
}

function cleanRunFile(runText) {
  return String(runText || "")
    .split(/\r?\n/)
    .filter(line => {
      const value = line.trim().toLowerCase();
      return !(
        value === "reset;" ||
        value.startsWith("model ") ||
        value.startsWith("data ") ||
        value.startsWith("option log_file ")
      );
    })
    .join("\n");
}

function extractSolverFromRun(runText) {
  const matches = [...String(runText || "").matchAll(/option\s+solver\s+([A-Za-z0-9_+-]+)\s*;/gi)];
  if (!matches.length) return null;
  return matches[matches.length - 1][1].trim().toUpperCase();
}

function xmlEscape(value) {
  return String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

async function neosXmlRpc(methodName, params = []) {
  const xmlParams = params.map(param => {
    if (Number.isInteger(param)) {
      return `<param><value><int>${param}</int></value></param>`;
    }

    return `<param><value><string>${xmlEscape(param)}</string></value></param>`;
  }).join("");

  const body = `<?xml version="1.0"?>
<methodCall>
  <methodName>${methodName}</methodName>
  <params>${xmlParams}</params>
</methodCall>`;

  const response = await fetch(NEOS_XMLRPC_URL, {
    method: "POST",
    headers: { "Content-Type": "text/xml" },
    body
  });

  const text = await response.text();

  if (!response.ok) {
    throw new Error(`XML-RPC HTTP ${response.status}: ${text}`);
  }

  const base64Match = text.match(/<base64>([\s\S]*?)<\/base64>/);
  if (base64Match) {
    return Buffer.from(base64Match[1].replace(/\s/g, ""), "base64");
  }

  const arrayMatch = text.match(/<array>[\s\S]*?<data>([\s\S]*?)<\/data>[\s\S]*?<\/array>/);
  if (arrayMatch) {
    return [...arrayMatch[1].matchAll(/<value>\s*<string>([\s\S]*?)<\/string>\s*<\/value>/g)]
      .map(match => match[1]);
  }

  const stringMatch = text.match(/<string>([\s\S]*?)<\/string>/);
  if (stringMatch) return stringMatch[1];

  const intMatch = text.match(/<int>([\s\S]*?)<\/int>|<i4>([\s\S]*?)<\/i4>/);
  if (intMatch) return Number(intMatch[1] || intMatch[2]);

  return text;
}

async function safeGetOutputFile(jobNumber, password, filename) {
  try {
    const file = await neosXmlRpc("getOutputFile", [
      Number(jobNumber),
      String(password),
      filename
    ]);

    return Buffer.isBuffer(file) ? file.toString("utf8") : String(file);
  } catch {
    return "";
  }
}

function cleanUsefulOutput(text) {
  const value = String(text || "").trim();

  if (!value) return "";

  if (
    value.toLowerCase().includes("could not retrieve") ||
    value.toLowerCase().includes("illegal file requested")
  ) {
    return "";
  }

  return value;
}

function chooseBestOutput({ resultsTxt, jobResults, amplOutput, jobOut }) {
  const candidates = [
    cleanUsefulOutput(amplOutput),
    cleanUsefulOutput(resultsTxt),
    cleanUsefulOutput(jobResults),
    cleanUsefulOutput(jobOut)
  ];

  return candidates.find(Boolean) || "No solver output was returned by NEOS.";
}

async function findNeosSolverInfo(solverName) {
  const raw = await neosXmlRpc("listAllSolvers", []);
  const lines = Array.isArray(raw) ? raw : String(raw).split(/\r?\n/);
  const solverUpper = String(solverName || "").toUpperCase();

  const matches = lines
    .map(line => String(line).trim())
    .filter(Boolean)
    .map(line => {
      const [category, solver, inputMethod] = line.split(":");
      return { category, solver, inputMethod };
    })
    .filter(item =>
      item.category &&
      item.solver &&
      item.inputMethod &&
      item.solver.toUpperCase() === solverUpper &&
      item.inputMethod.toUpperCase() === "AMPL"
    );

  return matches[0] || null;
}

// Submit to NEOS
app.post("/submit-neos", async (req, res) => {
  try {
    const email = String(req.body.email || "").trim();

    if (!email) {
      return res.status(400).json({
        error: "Email address is required for NEOS submission."
      });
    }

    if (!isValidEmail(email)) {
      return res.status(400).json({
        error: "Please enter a valid email address."
      });
    }

    const model = String(req.body.mod || "");
    const data = String(req.body.dat || "");
    const rawRun = String(req.body.run || "");

    if (!model || !data || !rawRun) {
      return res.status(400).json({
        error: "Please provide all three files: .mod, .dat, and .run."
      });
    }

    const commands = cleanRunFile(rawRun);
    const requestedSolver = extractSolverFromRun(rawRun);

    if (!requestedSolver) {
      return res.status(400).json({
        error: "Missing solver in .run file. Add a line like: option solver ipopt;"
      });
    }

    const solverInfo = await findNeosSolverInfo(requestedSolver);

    if (!solverInfo) {
      return res.status(400).json({
        error: `Could not find an AMPL-compatible NEOS solver entry for ${requestedSolver}.`,
        details: "Check that the solver name exists in NEOS for AMPL input."
      });
    }

    const { category, solver, inputMethod } = solverInfo;

    const xml = await NEOS.xmlstring({
      category,
      solver,
      inputMethod,
      email,
      model,
      data,
      commands
    });

    const job = await NEOS.submitJob(xml);

    if (
      !job.jobNumber ||
      Number(job.jobNumber) === 0 ||
      String(job.password || "").startsWith("Error")
    ) {
      return res.status(500).json({
        error: "NEOS submission failed.",
        details: job.password || "No valid job number returned from NEOS."
      });
    }

    jobs.set(String(job.jobNumber), {
      email,
      category,
      solver,
      inputMethod,
      model,
      data,
      run: rawRun,
      solverOutput: "",
      amplOutput: "",
      resultsTxt: "",
      jobResults: "",
      jobOut: "",
      emailSent: false,
      emailStatus: "",
      status: "Submitted"
    });

    console.log("NEOS job submitted");
    console.log("Email:", email);
    console.log("Category:", category);
    console.log("Solver:", solver);
    console.log("Input Method:", inputMethod);
    console.log("Job Number:", job.jobNumber);
    console.log("Password:", job.password);

    res.json({
      message: "NEOS job submitted successfully!",
      category,
      solver,
      inputMethod,
      email,
      jobNumber: job.jobNumber,
      password: job.password,
      status: "Submitted"
    });

  } catch (err) {
    console.error("NEOS Submission Error:", err);
    res.status(500).json({
      error: "Error submitting job to NEOS.",
      details: err.message
    });
  }
});

// Check NEOS job status
app.post("/neos-status", async (req, res) => {
  try {
    const { jobNumber, password } = req.body;

    if (!jobNumber || !password) {
      return res.status(400).json({
        error: "Job number and password are required."
      });
    }

    const status = await NEOS.getJobStatus(jobNumber, password);

    res.json({
      jobNumber,
      password,
      status
    });

  } catch (err) {
    res.status(500).json({
      error: "Error checking NEOS job status.",
      details: err.message
    });
  }
});

// Get NEOS results
app.post("/neos-results", async (req, res) => {
  try {
    const { jobNumber, password } = req.body;

    if (!jobNumber || !password) {
      return res.status(400).json({
        error: "Job number and password are required."
      });
    }

    const status = await NEOS.getJobStatus(jobNumber, password);
    const jobRecord = jobs.get(String(jobNumber));

    if (status !== "Done") {
      if (jobRecord) jobRecord.status = status;

      return res.json({
        jobNumber,
        password,
        status,
        output: `Job is not finished yet. Current status: ${status}`,
        emailStatus: "NEOS job is still running."
      });
    }

    const finalResults = await NEOS.getFinalResults(jobNumber, password);
    const amplOutput = finalResults.toString();
    const resultsTxt = await safeGetOutputFile(jobNumber, password, "results.txt");
    const jobResults = await safeGetOutputFile(jobNumber, password, "job.results");
    const jobOut = await safeGetOutputFile(jobNumber, password, "job.out");
    const output = chooseBestOutput({
      resultsTxt,
      jobResults,
      amplOutput,
      jobOut
    });

    let emailStatus = jobRecord?.emailStatus || "Email was not requested.";

    if (jobRecord && !jobRecord.emailSent) {
      try {
        await neosXmlRpc("emailJobResults", [
          Number(jobNumber),
          String(password)
        ]);
        jobRecord.emailSent = true;
        emailStatus = `NEOS was asked to email the completed results to ${jobRecord.email}.`;
      } catch (emailError) {
        emailStatus = `NEOS could not email the results: ${emailError.message}`;
        console.error("NEOS Email Error:", emailError);
      }
    }

    if (jobRecord) {
      jobRecord.status = status;
      jobRecord.solverOutput = output;
      jobRecord.amplOutput = amplOutput;
      jobRecord.resultsTxt = resultsTxt;
      jobRecord.jobResults = jobResults;
      jobRecord.jobOut = jobOut;
      jobRecord.emailStatus = emailStatus;
    }

    res.json({
      jobNumber,
      password,
      status,
      output,
      jobOut,
      emailStatus
    });

  } catch (err) {
    console.error("NEOS Results Error:", err);

    res.status(500).json({
      error: "Error retrieving NEOS results.",
      details: err.message
    });
  }
});

app.post("/download-zip", async (req, res) => {
  try {
    const { jobNumber } = req.body;

    if (!jobNumber) {
      return res.status(400).json({ error: "Job number is required." });
    }

    const jobRecord = jobs.get(String(jobNumber));

    if (!jobRecord) {
      return res.status(404).json({
        error: "Job files not found in memory. Please resubmit the job."
      });
    }

    const zip = new JSZip();

    zip.file("model.mod", jobRecord.model || "");
    zip.file("data.dat", jobRecord.data || "");
    zip.file("run.run", jobRecord.run || "");
    zip.file("solver_output.txt", jobRecord.solverOutput || jobRecord.amplOutput || "");
    zip.file("job.out", jobRecord.jobOut || "");

    const zipBuffer = await zip.generateAsync({ type: "nodebuffer" });

    res.set({
      "Content-Type": "application/zip",
      "Content-Disposition": `attachment; filename=optiweb-job-${jobNumber}.zip`
    });

    res.send(zipBuffer);

  } catch (err) {
    console.error("ZIP Download Error:", err);

    res.status(500).json({
      error: "Error creating ZIP file.",
      details: err.message
    });
  }
});

app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});
