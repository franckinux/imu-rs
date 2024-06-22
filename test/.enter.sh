# these two export lines are outside of the if block in order to enable creating
# the virtual env without warnings
export PIP_REQUIRE_VIRTUALENV=true
export PIP_USER=false

if [[ -f .venv/bin/activate ]]; then
   export PYTHONPATH=$(readlink -f .)

   source .venv/bin/activate
fi
