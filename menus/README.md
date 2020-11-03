# SMuFF Menus

These files are needed to build the menus. Copy this folder to your SMuFFs SD-Card.

If you're going to edit these files, please keep in mind:

- these files must be saved with UTF-8 encoding (because of the special characters).
- some lines may have a tab character (0x09) within and these **must not** be replaced by spaces.
- a single line starting with a dash symbol (-) will add an separator in the menu.
- some lines may contain C/C++ parameters/formatting symbols (%s, %d, %-8s...). These must not be altered and will be populated at runtime.
- menu entries may end with a **|nn** sequence. This is a internal function index and **must not** be changed.
- some menus may appear multiple times (such as main0, main1...). These reflect the menu variants.
