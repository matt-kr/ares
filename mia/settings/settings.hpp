struct Settings {
  auto serialize() -> string;
  auto unserialize(const string&) -> void;

  Boolean createManifests = false;
  Boolean useDatabase     = true;
  Boolean useHeuristics   = true;
#if defined(MIA_LIBRARY)
  string recent;
#else
  string recent           = Path::user();
#endif
};

extern Settings settings;
