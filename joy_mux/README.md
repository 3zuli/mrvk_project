# joy_mux

Subscribes to `source_topic`, publishes a list of topics defined by `published_topics`.
By default, source topic is redirected to the first topic in `published_topics` list.
Pressing the button defined by `button_index` will redirect the source topic to the next topic, wrapping around the end of the list.