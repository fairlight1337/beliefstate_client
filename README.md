### Beliefstate Client -- Deprecated

This is a legacy package that was deprecated in favor of the [the newer version](http://github.com/code-iai/semrec_client).

The original intent of these client libraries was (is) to make an easily accessible interface available for controlling the [Semantic Hierarchical Recorder](http://github.com/code-iai/semrec) (short semrec) infrastructure for robot memory logging.

Please update your code to use the newer version and adapt to the new namespace:

	namespace beliefstate_client {}

now became

	namespace semrec_client {}

The same applies for the main class, which now no longer is `BeliefstateClient`, but `SemrecClient`.