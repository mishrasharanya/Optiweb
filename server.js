const express = require("express");
const bodyParser = require("body-parser");
const fs = require("fs");
const NEOS = require("neos-js");

const app = express();
const PORT = 3000;

app.use(express.static("public"));
app.use(bodyParser.json());

function isValidEmail(email) {
  return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
}

// Generate .dat file
app.post("/generate", (req, res) => {
  try {
    fs.writeFileSync("test.dat", req.body.fileContent || "");
    res.send(".dat file generated and saved successfully!");
  } catch (err) {
    res.status(500).json({
      error: "Error saving test.dat",
      details: err.message
    });
  }
});

// Generate .mod file
app.post("/generate-mod", (req, res) => {
  try {
    fs.writeFileSync("test.mod", req.body.fileContent || "");
    res.send("test.mod saved successfully!");
  } catch (err) {
    res.status(500).json({
      error: "Error saving test.mod",
      details: err.message
    });
  }
});

// Generate .run file
app.post("/generate-run", (req, res) => {
  try {
    fs.writeFileSync("test.run", req.body.fileContent || "");
    res.send("test.run saved successfully!");
  } catch (err) {
    res.status(500).json({
      error: "Error saving test.run",
      details: err.message
    });
  }
});

// Submit to NEOS
app.post("/submit-neos", async (req, res) => {
  try {
    const category = req.body.category || "nco";
    const solver = req.body.solver || "IPOPT";
    const inputMethod = req.body.inputMethod || "AMPL";
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

    if (
      !fs.existsSync("test.mod") ||
      !fs.existsSync("test.dat") ||
      !fs.existsSync("test.run")
    ) {
      return res.status(400).json({
        error: "Missing one or more required files: test.mod, test.dat, or test.run."
      });
    }

    const model = fs.readFileSync("test.mod", "utf8");
    const data = fs.readFileSync("test.dat", "utf8");
    const commands = fs.readFileSync("test.run", "utf8");

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

    console.log("NEOS job submitted");
    console.log("Email:", email);
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

    if (status !== "Done") {
      return res.json({
        jobNumber,
        password,
        status,
        output: `Job is not finished yet. Current status: ${status}`
      });
    }

    const results = await NEOS.getFinalResults(jobNumber, password);
    const output = results.toString();

    res.json({
      jobNumber,
      password,
      status,
      output
    });

  } catch (err) {
    res.status(500).json({
      error: "Error retrieving NEOS results.",
      details: err.message
    });
  }
});

app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});