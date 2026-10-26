## =========================================================================== ##
# MIT License
# Copyright (c) 2026 JIC Smart Factory
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
## =========================================================================== ##
# Author   : Roman Parak
# File Name: ../test/Docs/test_citation.py
## =========================================================================== ##

# System (Default)
import os
# Datetime (Basic date and time types)
import datetime
# Regular expressions (Regular expression operations)
import re
# PyYAML (YAML parser and emitter) [pip3 install PyYAML]
import yaml

"""
Description:
    CITATION.cff against the Citation File Format 1.2.0 schema (the keys this file uses; the project pins no CFF validator,
    cffconvert 2.0.0 confirmed the file once, 2026-10-26) and against the repository: the README's title, the LICENSE, the
    README's repository URL, the authors of the README's Authors table.
"""
# The top-level keys of the CFF 1.2.0 schema, and the ones it requires.
#   Unit [Set<string>], [Set<string>]
CONST_CFF_KEYS = {'abstract', 'authors', 'cff-version', 'commit', 'contact', 'date-released', 'doi', 'identifiers', 'keywords',
                  'license', 'license-url', 'message', 'preferred-citation', 'references', 'repository', 'repository-artifact',
                  'repository-code', 'title', 'type', 'url', 'version'}
CONST_CFF_REQUIRED = {'authors', 'cff-version', 'message', 'title'}
# The person keys of the CFF 1.2.0 schema.
#   Unit [Set<string>]
CONST_PERSON_KEYS = {'address', 'affiliation', 'alias', 'city', 'country', 'email', 'family-names', 'fax', 'given-names',
                     'name-particle', 'name-suffix', 'orcid', 'post-code', 'region', 'tel', 'website'}


def _Citation(root):
    with open(os.path.join(root, 'CITATION.cff'), encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _Readme(root):
    with open(os.path.join(root, 'README.md'), encoding='utf-8') as handle:
        return handle.read()


def test_the_citation_follows_the_cff_1_2_0_schema(root):
    """
    Description:
        Only CFF 1.2.0 keys, the required ones present; cff-version 1.2.0, type software, the authors as persons with a given
        and a family name, version a string, date-released a date (YYYY-MM-DD), license an SPDX identifier, repository-code
        an https URL.
    """

    cff = _Citation(root)
    assert set(cff) <= CONST_CFF_KEYS, sorted(set(cff) - CONST_CFF_KEYS)
    assert CONST_CFF_REQUIRED <= set(cff)
    assert cff['cff-version'] == '1.2.0' and cff['type'] == 'software'
    assert isinstance(cff['message'], str) and cff['message'].strip()
    assert isinstance(cff['authors'], list) and cff['authors']
    for person in cff['authors']:
        assert set(person) <= CONST_PERSON_KEYS, person
        assert isinstance(person.get('given-names'), str) and isinstance(person.get('family-names'), str), person
    assert isinstance(cff['version'], str)
    assert isinstance(cff['date-released'], datetime.date)
    assert re.fullmatch(r'[A-Za-z0-9.+-]+', cff['license'])
    assert re.fullmatch(r'https://\S+', cff['repository-code'])


def test_the_citation_matches_the_repository(root):
    """
    Description:
        The title is the README's, version 1.0 (owner, 2026-10-26), the licence the LICENSE's MIT, the repository the one the
        README's release link names, and the authors those of the README's Authors table, in its order.
    """

    cff, readme = _Citation(root), _Readme(root)
    assert '# %s\n' % cff['title'] == readme.splitlines(keepends=True)[0]
    assert cff['version'] == '1.0'
    with open(os.path.join(root, 'LICENSE'), encoding='utf-8') as handle:
        assert cff['license'] == 'MIT' and handle.readline().strip() == 'MIT License'
    assert '%s/releases/tag/v%s' % (cff['repository-code'], cff['version']) in readme
    names = ['%s %s' % (p['given-names'], p['family-names']) for p in cff['authors']]
    shown = re.findall(r'<strong>([^<]+)</strong>', readme[readme.index('## Authors'):])
    assert names == shown
