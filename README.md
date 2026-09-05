# Optiweb

Optiweb is a browser-based AMPL and NEOS playground for building, previewing, submitting, and downloading optimization models. Users can upload or generate `.mod`, `.dat`, and `.run` files, inspect the AMPL text in the browser, submit jobs to NEOS, download result bundles, and view example trajectory visualizations with Three.js. The app is served as static files from `public/`; there is no Node/Express server required.

> [!WARNING]
> Optiweb is an incomplete research prototype and is still under active development. The included examples are useful for testing the workflow, but generated files and solver results must be reviewed before they are used for research conclusions, safety-critical systems, or production work.

## Live Application

The current Vercel deployment is available at:

**[https://optiweb-smoky.vercel.app/](https://optiweb-smoky.vercel.app/)**

The deployed application can be used without installing anything locally. Availability of optimization submissions still depends on the external NEOS service.

## Run Locally

Clone or download the repository, open a terminal in the project root, and start a static HTTP server:

```bash
cd OptiWeb
```

Open this URL in a browser:

```text
http://localhost:8000/public/index.html
```

Direct pages:

```text
http://localhost:8000/public/create.html
http://localhost:8000/public/try.html
http://localhost:8000/public/Manual.html
```

This project is currently static/client-side only.

## What Runs In The Browser

- File upload and preview for `.mod`, `.dat`, and `.run`
- AMPL file generation in `Manual.html`
- Demo loading from `public/demos/`
- Three.js trajectory visualization in `try.html`
- ZIP download creation through `public/clientZip.js`
- NEOS XML-RPC submission through `public/neosClient.js`

## Trying the Examples

The **Try** page includes browser-ready demonstrations for:

- Moving Block
- Cart-Pole
- Five-Linked Robot

Additional NEOS example file sets are available in `neos-server/`. Each example normally consists of matching `.mod`, `.dat`, and `.run` files. Current examples include:

- Moving Block
- Acrobot
- Cart-Pendulum
- Five-Link Robot
- Kinematic Car
- Grasp optimization with multiple solvers
- Spot jump with RK1, RK4, and collocation formulations

To try one of these additional examples through the interface:

1. Open `create.html` and select **Upload files**.
2. Choose the matching `.mod`, `.dat`, and `.run` files from `neos-server/`.
3. Review all three file previews.
4. Enter a valid email address and submit the job to NEOS.
5. Save the returned job number and password and wait for the result.

The solver named in the `.run` file must be available on NEOS and compatible with AMPL input. Do not submit the same job repeatedly while it is still running.

## Manual Builder

`Manual.html` provides a multi-step interface for grid, robot, constraint, Runge-Kutta, seed, custom data, and solver inputs. Robot, constraint, seed, and Runge-Kutta sections support pasted AMPL content as well as their other input methods.

The builder uses a shared trajectory-optimization model structure and generates problem-specific data. It is not yet guaranteed to reproduce every file in `neos-server/` without additional custom `.mod`, `.dat`, or `.run` content. In particular, complex cases such as Spot may require force initialization, bounds, contact scheduling, objective parameters, and specialized problem declarations.

## Current Limitations

- This is a research prototype, not a finished production application.
- The browser depends on the availability of NEOS.
- NEOS jobs can take time or fail because of model, data, solver, or service conditions.
- Large examples should be validated against their known `.mod`, `.dat`, and `.run` files.
- Browser-generated downloads and in-memory job information do not persist reliably across page refreshes.
