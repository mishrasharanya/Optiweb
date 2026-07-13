# Optiweb

Optiweb is a browser-based AMPL and NEOS playground for building, previewing, submitting, and downloading optimization models. Users can upload or generate `.dat`, `.mod`, and `.run` files, inspect the AMPL text in the browser, submit jobs to NEOS, download result bundles, and view example trajectory visualizations with Three.js. The app is served as static files from `public/`; there is no Node/Express server required.

## Run Locally

From the project root:

```bash
python3 -m http.server 8000
```

Then open:

```text
http://localhost:8000/public/index.html
```

Direct pages:

```text
http://localhost:8000/public/create.html
http://localhost:8000/public/try.html
http://localhost:8000/public/Manual.html
```

this project is currently static/client-side only.

## What Runs In The Browser

- File upload and preview for `.dat`, `.mod`, and `.run`
- AMPL file generation in `Manual.html`
- Demo loading from `public/demos/`
- Three.js trajectory visualization in `try.html`
- ZIP download creation through `public/clientZip.js`
- NEOS XML-RPC submission through `public/neosClient.js`

