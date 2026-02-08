namespace nonlin {
class offboard_common {
  public:
    offboard_common();
    ~offboard_common() = default;

  private:
    offboard_enable(bool enable);
    offboard_keep_alive();
    offboard_log();

    // Maybe list experiments here?
};
}; // namespace nonlin