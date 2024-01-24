SOURCES = \
	exercise-1 \
	exercise-2 \
	exercise-3 \
	exercise-4

SOURCES_ASCIIDOC=$(SOURCES:%=%.asciidoc)
HTML_FILES=$(SOURCES_ASCIIDOC:%.asciidoc=%.html)

all: $(HTML_FILES)

%.html: %.asciidoc
	asciidoctor -b html -o $@ $<

clean:
	rm -f $(HTML_FILES)
