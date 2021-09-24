# Host miscellaneous communication manager
This component is intended to expose certain miscellaneous host interface
communication mechanisms like `PLTRST`, `Core BIOS Done` & `MailBox`
registers.

## Overview
In OpenBMC, miscellaneous host interface communications like `PLTRST`,
`Core BIOS Done`, & `MailBox` registers must be used by different
applications. As OpenBMC relies on D-Bus, the same needs to be exposed in D-Bus,
so that single application can monitor it from driver end, and propagate
details to many applications.

## Capabilities
`host-misc-comm-manager` will monitor the `PLTRST`, `Core BIOS Done`, &
`MailBox` registers from it's corresponding driver and expose the same
in it's own D-Bus object & properties.

Following objects with interface & properties mentioned below will be exposed

1. Object under `/xyz/openbmc_project/misc/platform_state` will expose
`xyz.openbmc_project.State.Host.Misc` interface with 2 boolean type read-only
properties`ESpiPlatformReset` & `CoreBiosDone`.

2. Object under `/xyz/openbmc_project/misc/mailbox/<register>` will expose
`xyz.openbmc_project.Misc.Mailbox` interface and under which uint8_t value
of property named `Value` will be exposed.
There will be `n` number of similar objects based on the mailbox size.
Application can update the register value by modifying `Value` property.

### eSPI platform reset
eSPI platform reset can be used to determine the host reset.

### Core BIOS Done
Used to indicate that UEFI has finished executing Core DXE Phase, and started
dispatching external OpROM code. Any security feature like KCS Trust policy
enforcement must be enabled at this point of time.

Note: For enhanced security 600 seconds timeout is implemented, beyond which
BMC will assume that BIOS has finished Core DXE Phase.

UEFI & BMC can use any agreed method to indicate `CoreBiosDone` as long as
external OpROM can not revert, before BMC detects it. Products with AST2500 /
AST2600 BMC chip must use the SIO software interrupt register, which provides a
way for BMC to detect that UEFI has set this bit already, even if external OpROM
tries to clear the same. This interrupt bit can not be cleared from UEFI end and
can only be cleared from BMC end. The register location differs between AST2500
and AST2600, so UEFI firmware for supported products must use the correct one to
indicate `CoreBiosDone`:

- AST2500: `SIORx_20[0]`
- AST2600: `SIORx_21[0]`

### Mailbox registers
Mailbox registers under AST2500 / AST2600, will be exposed as D-Bus objects.
These registers are used for communication with Host & BMC. Individual register
usage is beyond the scope of this component.