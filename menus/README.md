# SMuFF Menus

These files are used to build the menus. Copy this folder to your SMuFFs SD-Card.

If you're going to edit these files, please keep in mind:

- these files must be saved with UTF-8 encoding.
- some lines may contain a tab character (0x09) and **must not** be replaced by spaces.
- a single line starting with a dash symbol (-) will add an separator in the menu.
- some lines may contain C/C++ parameters/formatting symbols (%s, %d, %-8s...). These **must not** be altered and will be populated at runtime.
- menu entries may end with a **|nn** sequence. This is a internal function index and **must not** be changed.
- some menus appear multiple times (such as main0, main1...). These reflect the menu variants.
