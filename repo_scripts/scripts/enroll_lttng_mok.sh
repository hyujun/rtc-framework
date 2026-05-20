#!/usr/bin/env bash
# enroll_lttng_mok.sh — sign lttng-modules DKMS build with a Machine Owner Key
# so a Secure-Boot-enabled kernel will accept the module.
#
# Symptom this fixes:
#   modprobe lttng-tracer
#   → "Key was rejected by service"
#   lttng list --kernel
#   → "Error: Unable to list kernel events: Kernel tracer not available"
#
# Cause:
#   DKMS rebuilds lttng-tracer.ko on each kernel install, but the resulting
#   module is unsigned. Secure Boot rejects unsigned kernel modules. The fix
#   is to generate an MOK (Machine Owner Key), enroll it in the firmware via
#   the UEFI MOK Manager (shim) at the next reboot, then have DKMS sign all
#   future module builds with that key.
#
# This script is idempotent: re-runs detect existing keys, existing MOK
# enrollment, and existing DKMS configuration, and skip the matching steps.
#
# Usage:
#   ./repo_scripts/scripts/enroll_lttng_mok.sh
#
# Flow:
#   1. Generate /var/lib/shim-signed/mok/MOK.{priv,der} if missing.
#   2. Tell DKMS to sign with that key via /etc/dkms/framework.conf.d/.
#   3. Rebuild lttng-modules so the .ko gets signed.
#   4. Submit the public key to mokutil; user picks a one-time password.
#   5. Print clear next-steps: reboot, MOK Manager → Enroll MOK → reboot.
#
# After reboot + enrollment:
#   sudo modprobe lttng-tracer
#   lsmod | grep lttng    # should now show lttng_tracer
#   sudo systemctl restart lttng-sessiond
#   lttng list --kernel   # should now list available kernel events
#
# Re-running this script is safe — it skips already-completed work and
# prints diagnostics if Secure Boot is off or already-trusted keys cover
# the DKMS module.

set -euo pipefail

# ── Style ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; YELLOW='\033[1;33m'; GREEN='\033[0;32m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${BOLD}[mok]${NC} $*"; }
warn()    { echo -e "${YELLOW}[mok WARN]${NC} $*"; }
err()     { echo -e "${RED}[mok ERROR]${NC} $*" >&2; }
success() { echo -e "${GREEN}[mok OK]${NC} $*"; }

KEY_DIR="/var/lib/shim-signed/mok"
KEY_PRIV="${KEY_DIR}/MOK.priv"
KEY_CERT="${KEY_DIR}/MOK.der"
FRAMEWORK="/etc/dkms/framework.conf.d/lttng-modules-sign.conf"
KERNEL_RELEASE="$(uname -r)"

# ── Pre-flight ───────────────────────────────────────────────────────────────
require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    err "missing required command: $1 (sudo apt install $2)"
    exit 1
  fi
}
require_cmd mokutil mokutil
require_cmd dkms dkms
require_cmd openssl openssl

if ! sb_state="$(mokutil --sb-state 2>/dev/null)"; then
  err "mokutil --sb-state failed — Secure Boot status unknown"
  exit 1
fi
if [[ "$sb_state" != *"SecureBoot enabled"* ]]; then
  warn "Secure Boot is NOT enabled — this script is unnecessary."
  warn "  $sb_state"
  warn "If modprobe lttng-tracer still fails, the cause is elsewhere."
  exit 0
fi
info "$sb_state"

# Confirm the DKMS package is at least installed.
if ! dkms status 2>&1 | grep -q "^lttng-modules"; then
  err "lttng-modules DKMS package not installed."
  err "  Run: ./install.sh --tracing  (or: sudo apt install lttng-modules-dkms)"
  exit 1
fi
info "DKMS package: $(dkms status | grep lttng-modules | head -1)"

# ── Step 1: generate MOK key pair if missing ─────────────────────────────────
if [[ -f "$KEY_PRIV" && -f "$KEY_CERT" ]]; then
  success "MOK key pair already present at ${KEY_DIR}/"
else
  info "Generating MOK key pair at ${KEY_DIR}/..."
  sudo mkdir -p "$KEY_DIR"
  sudo openssl req -new -x509 -newkey rsa:2048 -nodes -days 36500 \
      -subj "/CN=$(hostname) DKMS MOK/" \
      -keyout "$KEY_PRIV" -out "$KEY_CERT" 2>/dev/null
  sudo chmod 600 "$KEY_PRIV"
  sudo chmod 644 "$KEY_CERT"
  success "Generated MOK.priv (kept private) and MOK.der (public cert)"
fi

# ── Step 2: tell DKMS to sign with this key ──────────────────────────────────
if [[ -f "$FRAMEWORK" ]] \
   && grep -q "^mok_signing_key=$KEY_PRIV" "$FRAMEWORK" \
   && grep -q "^mok_certificate=$KEY_CERT"  "$FRAMEWORK"; then
  success "DKMS already configured to sign with this MOK key"
else
  info "Writing DKMS sign config → $FRAMEWORK"
  sudo tee "$FRAMEWORK" >/dev/null <<EOF
# Written by enroll_lttng_mok.sh — sign DKMS modules with the Machine Owner Key
# so Secure Boot will accept them.
mok_signing_key="$KEY_PRIV"
mok_certificate="$KEY_CERT"
sign_tool="/usr/lib/linux-kbuild-$(uname -r | sed 's/-[^-]*$//')/scripts/sign-file"
EOF
  success "DKMS framework configured"
fi

# ── Step 3: rebuild lttng-modules so the .ko gets re-signed ──────────────────
info "Rebuilding lttng-modules against kernel ${KERNEL_RELEASE} (will sign on success)..."
if sudo dkms autoinstall -k "$KERNEL_RELEASE" 2>&1 | tee /tmp/lttng-dkms-rebuild.log | tail -20; then
  if grep -q "Signing module" /tmp/lttng-dkms-rebuild.log; then
    success "Module rebuilt and signed"
  else
    warn "Rebuild completed but no 'Signing module' line in DKMS output."
    warn "Check /tmp/lttng-dkms-rebuild.log — the sign step may have been skipped."
  fi
else
  err "DKMS rebuild failed — see /tmp/lttng-dkms-rebuild.log"
  exit 1
fi

# ── Step 4: enrol the public key in firmware (mokutil import) ────────────────
# mokutil --test-key returns 0 if the key is already enrolled in MokList /
# .platform. Skip the password prompt if so.
if sudo mokutil --test-key "$KEY_CERT" 2>&1 | grep -q "already enrolled"; then
  success "MOK already enrolled in firmware — nothing more to do."
  echo ""
  info "Try now:"
  info "  sudo modprobe lttng-tracer && lsmod | grep lttng"
  info "  sudo systemctl restart lttng-sessiond"
  info "  lttng list --kernel"
  exit 0
fi

info "Submitting public key to mokutil for enrolment..."
warn "You will be asked to set a ONE-TIME password."
warn "Memorise it — you must type it back during the next boot's MOK dialog."
sudo mokutil --import "$KEY_CERT"

# ── Step 5: tell the user what happens next ──────────────────────────────────
echo ""
success "MOK enrolment request submitted."
echo ""
info "Next steps (cannot be automated — UEFI firmware UI):"
info "  1. Reboot the machine."
info "  2. A blue/black 'Perform MOK management' screen appears (10-second timeout)."
info "  3. Select 'Enroll MOK' → 'Continue' → 'Yes' → type the password you set above."
info "  4. Select 'Reboot'."
info "  5. After login, verify:"
info "       sudo modprobe lttng-tracer"
info "       lsmod | grep lttng                        # should show lttng_tracer"
info "       sudo systemctl restart lttng-sessiond"
info "       lttng list --kernel                       # should list events"
info "  6. If MOK dialog does not appear, re-run this script and reboot again."
