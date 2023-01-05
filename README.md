# BEWARE THE ~o~ HYPNOSQUID

## *Text is work in progress*

### Philosophy

The engine will follow a philosophy akin to table-top role-playing games (TRPGs). In such a game,
you have multiple peopling keeping track and communicating about the game world. In engine terms communication is done
through Events,
people are represented by one or multiple Systems, and they all reason around the World.

### Engine Extensions

Core engine extensions are built into the engine library, and as such they are enforced on anyone who
wish to use the engine. This means these extensions need to very clearly be motivated according to the TRPG philosophy.
As an example, the World extension keeps track of the imaginary world laid out in such a game. A counter example is
3D rendering, the world can be conveyed in many different ways, as a board, or simply through speech, and should as such
be kept purely on the system level.

### Systems

Systems are simple functions which operate on different parts of the engine through their parameters. This way of
explicitly
stating system requirements allows the engine to run concurrently without any unexpected world changes.
Going back to the TRPG analogy, if there were a real game board, two people might want to move their pieces at the same
time,
though moving the same piece at the same time proves impossible.

### Plugins

Plugins are dynamically linked libraries which are able to operate on the engine object at runtime. They are specified
either explicitly in the Engine constructor, or more dynamically through an engine configuration file.