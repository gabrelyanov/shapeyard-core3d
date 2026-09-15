# Shared by guarded OCCT workers. The caller supplies die().
verify_disk_guard_ancestry() {
  local protocol guard_pid supervisor_pid advertised_pgid
  local worker_pgid supervisor_parent lock_owner guard_command

  protocol="${SHAPEYARD_DISK_GUARD_PROTOCOL:-}"
  guard_pid="${SHAPEYARD_DISK_GUARD_PID:-}"
  supervisor_pid="${SHAPEYARD_DISK_GUARD_SUPERVISOR_PID:-}"
  advertised_pgid="${SHAPEYARD_DISK_GUARD_PGID:-}"
  [[ "${protocol}" == "v1" \
      && "${guard_pid}" =~ ^[0-9]+$ \
      && "${supervisor_pid}" =~ ^[0-9]+$ \
      && "${advertised_pgid}" =~ ^[0-9]+$ ]] \
    || die "the OCCT worker is missing live disk-guard provenance"
  [[ "${supervisor_pid}" == "${advertised_pgid}" \
      && "${PPID}" == "${supervisor_pid}" ]] \
    || die "the OCCT worker is outside the expected guard supervisor"

  worker_pgid="$(/bin/ps -o pgid= -p "$$")"
  worker_pgid="${worker_pgid//[[:space:]]/}"
  supervisor_parent="$(/bin/ps -o ppid= -p "${supervisor_pid}")"
  supervisor_parent="${supervisor_parent//[[:space:]]/}"
  [[ "${worker_pgid}" == "${advertised_pgid}" \
      && "${supervisor_parent}" == "${guard_pid}" ]] \
    || die "the OCCT worker process group or ancestry is not guard-owned"
  kill -0 "${guard_pid}" 2>/dev/null \
    && kill -0 "${supervisor_pid}" 2>/dev/null \
    || die "the OCCT disk guard or supervisor is no longer alive"

  lock_owner="$(sed -n '1p' /private/tmp/.shapeyard-disk-budget-global.lock 2>/dev/null || true)"
  [[ "${lock_owner}" == "${guard_pid}" ]] \
    || die "the live disk-guard lock does not match the OCCT worker ancestry"
  guard_command="$(/bin/ps -o command= -p "${guard_pid}")"
  [[ "${guard_command}" == *"with_disk_budget.sh"* ]] \
    || die "the advertised OCCT guard process is not the canonical disk guard"

  unset SHAPEYARD_DISK_GUARD_PROTOCOL
  unset SHAPEYARD_DISK_GUARD_PID
  unset SHAPEYARD_DISK_GUARD_SUPERVISOR_PID
  unset SHAPEYARD_DISK_GUARD_PGID
}
