const API_URL = "http://" + window.location.hostname + ":8000";

// DOM Elements
const busyDot = document.getElementById('busy-dot');
const busyText = document.getElementById('busy-text');
const ikDot = document.getElementById('ik-dot');
const ikText = document.getElementById('ik-text');

const alphaVal = document.getElementById('alpha-val');
const betaVal = document.getElementById('beta-val');
const xVal = document.getElementById('x-val');
const zVal = document.getElementById('z-val');

const absForm = document.getElementById('abs-control-form');
const targetXInput = document.getElementById('target-x');
const targetZInput = document.getElementById('target-z');
const toggleIkBtn = document.getElementById('toggle-ik-btn');

const allButtons = document.querySelectorAll('button');

let isArmBusy = false;

const WS_URL = "ws://" + window.location.hostname + ":8000/ws";

let socket;
function connectWebSocket() {
    socket = new WebSocket(WS_URL);

    socket.onmessage = function (event) {
        const data = JSON.parse(event.data);

        // Update Telemetry
        alphaVal.innerHTML = `${data.alpha}&deg;`;
        betaVal.innerHTML = `${data.beta}&deg;`;
        xVal.innerText = data.X.toFixed(2);
        zVal.innerText = data.Z.toFixed(2);

        // Update Busy State
        isArmBusy = data.is_busy;
        if (isArmBusy) {
            busyDot.classList.remove('active');
            busyDot.style.backgroundColor = 'var(--danger)';
            busyDot.style.boxShadow = '0 0 10px var(--danger)';
            busyText.innerText = 'Busy';
        } else {
            busyDot.classList.add('active');
            busyDot.style.backgroundColor = 'var(--accent)';
            busyDot.style.boxShadow = '0 0 10px var(--accent)';
            busyText.innerText = 'Idle';
        }

        // Update IK Toggle State
        if (data.ik_toggle) {
            ikDot.classList.add('active');
            ikDot.style.backgroundColor = 'var(--accent)';
            ikDot.style.boxShadow = '0 0 10px var(--accent)';
            ikText.innerText = 'IK ON';
        } else {
            ikDot.classList.remove('active');
            ikDot.style.backgroundColor = 'var(--text-secondary)';
            ikDot.style.boxShadow = 'none';
            ikText.innerText = 'IK OFF';
        }

        // Disable buttons if busy
        allButtons.forEach(btn => btn.disabled = isArmBusy);
    };

    socket.onclose = function () {
        console.warn("WebSocket disconnected. Reconnecting...");
        setTimeout(connectWebSocket, 1000);
    };
}

connectWebSocket();

function showToast(message, isError = false) {
    const toast = document.getElementById('toast');
    toast.innerText = message;
    toast.style.borderColor = isError ? 'var(--danger)' : 'var(--accent)';
    toast.style.color = isError ? 'var(--danger)' : 'var(--accent)';

    toast.classList.remove('hidden');
    // small delay for css transition
    setTimeout(() => toast.classList.add('show'), 10);

    setTimeout(() => {
        toast.classList.remove('show');
        setTimeout(() => toast.classList.add('hidden'), 300);
    }, 3000);
}

// Absolute Control Submit
absForm.addEventListener('submit', async (e) => {
    e.preventDefault();
    if (isArmBusy) return;

    const x = parseFloat(targetXInput.value);
    const z = parseFloat(targetZInput.value);

    if (isNaN(x) || isNaN(z)) {
        showToast("Please enter valid numbers for X and Z", true);
        return;
    }

    try {
        const response = await fetch(`${API_URL}/move/ik`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ X: x, Z: z })
        });

        const data = await response.json();
        if (!response.ok) {
            showToast(data.detail || "Error moving arm", true);
        } else {
            showToast(`Moving to X: ${x}, Z: ${z}`);
        }
    } catch (error) {
        showToast("Network error", true);
    }
});

// Toggle IK
toggleIkBtn.addEventListener('click', async () => {
    if (isArmBusy) return;
    try {
        const response = await fetch(`${API_URL}/ik-toggle`, { method: 'POST' });
        if (!response.ok) throw new Error("Failed to toggle IK");
        showToast("IK Toggled");
    } catch (error) {
        showToast(error.message, true);
    }
});

// Relative Controls
const sendRelativeMove = async (deltaX, deltaZ) => {
    if (isArmBusy) return;
    try {
        const response = await fetch(`${API_URL}/move/relative`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ delta_X: deltaX, delta_Z: deltaZ })
        });
        const data = await response.json();
        if (!response.ok) {
            showToast(data.detail || "Error moving arm", true);
        } else {
            showToast(`Moving relative dX: ${deltaX}, dZ: ${deltaZ}`);
        }
    } catch (error) {
        showToast("Network error", true);
    }
};

document.getElementById('x-up-btn').addEventListener('click', () => sendRelativeMove(100, 0));
document.getElementById('x-down-btn').addEventListener('click', () => sendRelativeMove(-100, 0));
document.getElementById('z-up-btn').addEventListener('click', () => sendRelativeMove(0, 100));
document.getElementById('z-down-btn').addEventListener('click', () => sendRelativeMove(0, -100));
