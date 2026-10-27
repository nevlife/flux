_flux() {
  mapfile -t COMPREPLY < <(flux __complete "${COMP_WORDS[@]:1:COMP_CWORD}" 2>/dev/null)
}
complete -F _flux flux
