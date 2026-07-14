function isValidEmail(email) {
  return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
}

const NEOS_XMLRPC_URL = "https://neos-server.org:3333";

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
    const rawBinaryString = atob(base64Match[1].replace(/\s/g, ""));
    return rawBinaryString; 
  }

  const arrayMatch = text.match(/<array>[\s\S]*?<data>([\s\S]*?)<\/data>[\s\S]*?<\/array>/);
  if (arrayMatch) {
    return [...arrayMatch[1].matchAll(/<value>\s*<(string|int|i4)>([\s\S]*?)<\/\1>\s*<\/value>/g)]
      .map(match => match[2].trim()); 
  }

  const stringMatch = text.match(/<string>([\s\S]*?)<\/string>/);
  if (stringMatch) return stringMatch[1];

  const intMatch = text.match(/<int>([\s\S]*?)<\/int>|<i4>([\s\S]*?)<\/i4>/);
  if (intMatch) return Number(intMatch[1] || intMatch[2]);

  return text;
}

const jobs = new Map();

async function safeGetOutputFile(jobNumber, password, filename) {
  try {
    const file = await neosXmlRpc("getOutputFile", [
      Number(jobNumber),
      String(password),
      filename
    ]);

    return file ? String(file) : "";
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

async function submitNeosJob({ email, mod, dat, run }) {
  const cleanEmail = String(email || "").trim();

  if (!cleanEmail) {
    throw new Error("Email address is required for NEOS submission.");
  }

  if (!isValidEmail(cleanEmail)) {
    throw new Error("Please enter a valid email address.");
  }

  const model = String(mod || "");
  const data = String(dat || "");
  const rawRun = String(run || "");

  if (!model || !data || !rawRun) {
    throw new Error("Please provide all three files: .mod, .dat, and .run.");
  }

  const commands = cleanRunFile(rawRun);
  const requestedSolver = extractSolverFromRun(rawRun);

  if (!requestedSolver) {
    throw new Error("Missing solver in .run file. Add a line like: option solver ipopt;");
  }

  const solverInfo = await findNeosSolverInfo(requestedSolver);

  if (!solverInfo) {
    throw new Error(`Could not find an AMPL-compatible NEOS solver entry for ${requestedSolver}. Check that the solver name exists in NEOS for AMPL input.`);
  }

  const { category, solver, inputMethod } = solverInfo;

  const xmlPayloadString = `<document>
<category>${category}</category>
<solver>${solver}</solver>
<inputMethod>${inputMethod}</inputMethod>
<email>${xmlEscape(cleanEmail)}</email>
<model>${xmlEscape(model)}</model>
<data>${xmlEscape(data)}</data>
<commands>${xmlEscape(commands)}</commands>
</document>`.trim();

  const jobResultArray = await neosXmlRpc("submitJob", [xmlPayloadString]);

  const jobNum = (jobResultArray && jobResultArray[0]) ? Number(jobResultArray[0]) : 0;
  const jobPass = (jobResultArray && jobResultArray[1]) ? String(jobResultArray[1]) : "";

  if (jobNum === 0 || jobPass.startsWith("Error")) {
    throw new Error(jobPass || "No valid job number returned from NEOS.");
  }

  jobs.set(String(jobNum), {
    email: cleanEmail,
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

  console.log("[NEOS Native Client] Job Registered:", jobNum);

  return {
    message: "NEOS job submitted successfully!",
    category,
    solver,
    inputMethod,
    email: cleanEmail,
    jobNumber: jobNum,
    password: jobPass,
    status: "Submitted"
  };
}

async function getNeosJobStatus(jobNumber, password) {
  if (!jobNumber || !password) {
    throw new Error("Job number and password are required.");
  }
  
  const status = await neosXmlRpc("getJobStatus", [Number(jobNumber), String(password)]);
  return status;
}

async function getNeosResults(jobNumber, password) {
  if (!jobNumber || !password) {
    throw new Error("Job number and password are required.");
  }

  const status = await neosXmlRpc("getJobStatus", [Number(jobNumber), String(password)]);
  const jobRecord = jobs.get(String(jobNumber));

  if (status !== "Done") {
    if (jobRecord) jobRecord.status = status;

    return {
      jobNumber,
      password,
      status,
      output: `Job is not finished yet. Current status: ${status}`,
      emailStatus: "NEOS job is still running."
    };
  }

  const amplOutput = await neosXmlRpc("getFinalResultsByPassword", [Number(jobNumber), String(password)]);
  
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

  return {
    jobNumber,
    password,
    status,
    output,
    amplOutput,
    resultsTxt,
    jobResults,
    jobOut,
    emailStatus
  };
}

async function downloadJobZip(jobNumber) {
  if (!jobNumber) {
    throw new Error("Job number is required.");
  }

  const jobRecord = jobs.get(String(jobNumber));

  if (!jobRecord) {
    throw new Error("Job files not found in browser client memory. Please resubmit the job.");
  }

  if (typeof window.JSZip === 'undefined') {
    throw new Error("JSZip library has not loaded yet. Verify your HTML CDN script tag.");
  }
  
  const zip = new window.JSZip();

  zip.file("model.mod", jobRecord.model || "");
  zip.file("data.dat", jobRecord.data || "");
  zip.file("run.run", jobRecord.run || "");
  zip.file("solver_output.txt", jobRecord.solverOutput || jobRecord.amplOutput || "");
  zip.file("job.out", jobRecord.jobOut || "");

  const zipBlob = await zip.generateAsync({ type: "blob" });

  const link = document.createElement("a");
  link.href = URL.createObjectURL(zipBlob);
  link.download = `optiweb-job-${jobNumber}.zip`;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  URL.revokeObjectURL(link.href);
  
  return { success: true };
}

window.submitNeosJob = submitNeosJob;
window.getNeosResults = getNeosResults;
window.downloadJobZip = downloadJobZip;