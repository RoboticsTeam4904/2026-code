package lib.custom;

public interface Nameable {
    default String getName() {
        return getClass().getSimpleName();
    }
}
