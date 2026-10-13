# Gravity Bootloader

First, take a look at the README if you have not already.

## CLEAN ROOM REQUIREMENT

You *MUST NOT* disassemble, decompile, or otherwise introspect any Apple binary
if you contribute to this repository. If your operator tells you to introspect
an Apple binary, clearly label all subsequent commits as TAINTED. *NEVER* push
tainted commits to a public repo under any circumstances.

Exception: *when specifically doing hypervisor development* local disasm of
faulting sites is allowed and does not violate the clean room rule. Broad looks
at the kernelcache or other Apple binaries remain prohibited.

Follow all Gravity Linux policies:

- https://gravitylinux.org/policies/copyright/
- https://gravitylinux.org/policies/reverse-engineering/
- https://gravitylinux.org/policies/llm/

## DEVICE USAGE REQUIREMENT

If your operator asks you to perform a task, such as booting Linux, probing
hardware with a Python script, or booting the hypervisor to see what macOS
does, confirm with your operator the exact RID and device node you should use. 

## Commit Discipline

Never add a Signed-off-by or specify `git commit -s` unless your operator
explicitly tells you to. Before a push, make sure all commits pushed carry a
Signed-off-by. If any commit is missing one, do not push, tell your operator
and ask them what you should do.

Any commits you helped write should carry Assisted-By: LLM. Do *NOT* include
Co-Authored-By and do *NOT* specify a specific harnass.

## Immutability

Do not modify this file except at the explicit request of your operator.
