;; This file is part of MutekH.
;;
;; MutekH is free software; you can redistribute it and/or modify it
;; under the terms of the GNU Lesser General Public License as
;; published by the Free Software Foundation; version 2.1 of the
;; License.
;;
;; MutekH is distributed in the hope that it will be useful, but
;; WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;; Lesser General Public License for more details.
;;
;; You should have received a copy of the GNU Lesser General Public
;; License along with this program.  If not, see
;; <http://www.gnu.org/licenses/>.
;;
;; Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>

(defvar bc-mode-hook nil)
(defvar bc-basic-indent 4 "Bytecode indent width.")

(setq auto-mode-alist
      (append '(("\\.bc\\'" . bc-mode)) auto-mode-alist)
      )

(defconst bc-font-lock-keywords
  (list
   ; Builtins
   '("\\.\\(backend\\|c\\(lobber\\|onst\\|ustom\\)\\|d\\(ata16\\|efine\\)\\|e\\(ntry\\|xport\\)\\|global\\|input\\|name\\|output\\|preserve\\)" . font-lock-builtin-face)
   ; Preprocessed values
   '("\\(_\\(const\\|offsetof\\|sizeof\\)\\|bitpos\\|#define\\)" . font-lock-preprocessor-face)
   ; jmp[...] target
   '("\\b\\(jmp\\(?:8\\|32\\)\\)[ \t]+\\(\\(?:\\s_\\|\\sw\\)+\\)"
     (1 font-lock-builtin-face)
     (2 font-lock-reference-face))
   ; %label:name
   '("\\(%\\)\\(\\(?:\\s_\\|\\sw\\)+\\):\\(\\(?:\\s_\\|\\sw\\)+\\)"
     (1 font-lock-variable-name-face)
     (2 font-lock-function-name-face)
     (3 font-lock-variable-name-face))
   ; .func label / .export label
   '("\\(\\.\\(?:func\\|export\\)\\)[ \t]+\\(\\(?:\\s_\\|\\sw\\)+\\)"
     (1 font-lock-keyword-face)
     (2 font-lock-function-name-face t))
   ; .endfunc
   '("\\(\\.endfunc\\)" 1 font-lock-keyword-face)
   ; %n name register aliasing
   '("\\(%\\)\\([0-9]+\\)[ \t]+\\(\\(?:\\s_\\|\\sw\\)+\\)\\b"
     (1 font-lock-variable-name-face t)
     (2 font-lock-variable-name-face t)
     (3 font-lock-variable-name-face t))
   ; %register
   '("%\\(\\(?:\\s_\\|\\sw\\)+\\)\\b" . font-lock-variable-name-face)
   ; label:
   '("^\\(\\(?:\\s_\\|\\sw\\)+\\):" . font-lock-reference-face)
   ; call %label:var, label
   '("\\bcall\\(?:8\\|32\\)[^,]+,[ \t]*\\(\\(?:\\s_\\|\\sw\\)+\\)" 1 font-lock-function-name-face t)
   ; loop ..., label
   '("\\bloop[^,]+,[ \t]*\\(\\(?:\\s_\\|\\sw\\)+\\)" 1 font-lock-reference-face t)
   ; #preprocessor
   '("^#\\(if\\|warning\\|error\\|endif\\|elif\\|include\\)" . font-lock-preprocessor-face)
   ; builtins
   '("\\b\\(a\\(bort\\|dd8?\\|ndn?32\\)\\|bit32[cs]\\|c\\(all\\(32\\|8\\)\\|call\\|st\\(8\\|16\\|32\\|64\\)\\)\\|d\\(ie\\|ump\\)\\|e\\(nd\\|q0?\\|xt[sz]\\)\\|gaddr\\|jmp\\(8\\|32\\)\\|l\\(addr\\(16\\|32\\)\\|d\\(8\\|16\\|32\\|64\\)[ei]?\\|t\\(eq\\)?s?\\)\\|m\\(ov\\|sbs32\\|ul32\\)\\|n\\(e\\(g\\|q0?\\)\\|op\\|ot32\\)\\|or32\\|pack\\(8\\|\\(16\\|32\\)\\(le\\|be\\)\\)\\|ret\\|sh\\(i32[lr]\\|[lr]32\\)\\|s\\(t\\(8\\|16\\|32\\|64\\)[ei]?\\|ub\\|wap\\(16\\|32\\)\\(le\\|be\\)?\\)\\|t\\(race\\|st32[cs]\\)\\|unpack\\(8\\|\\(16\\|32\\)\\(le\\|be\\)\\)\\|xor32\\)\\b" 1 font-lock-builtin-face)
   ; ALL_CAPS_CONSTANTS
   '("[A-Z_][A-Z0-9_]+" . font-lock-constant-face)
   ; #include "something"
   '("^#include \\(.*\\)$" (1 font-lock-string-face t))
   )
  )

(defvar bc-syntax-table
  (let ((st (make-syntax-table)))
    ; C-style comments
    (modify-syntax-entry ?/ ". 124" st)
    (modify-syntax-entry ?* ". 23b" st)
    (modify-syntax-entry ?\n ">" st)
    st))

(defun bc-indent-line ()
  "Auto-indent the current line."
  (interactive)
  (let* ((savep (point))
         (indent (condition-case nil
                     (save-excursion
                       (forward-line 0)
                       (skip-chars-forward " \t")
                       (if (>= (point) savep) (setq savep nil))
                       (max (bc-calculate-indentation) 0))
                   (error 0))))
    (if savep (save-excursion (indent-line-to indent)) (indent-line-to indent))))

(defun bc-calculate-indentation ()
  (or
   ;; labels
   (and (looking-at "\\(\\sw\\|\\s_\\)+:\\s-*$") 0)
   ;; #preprocessor directives
   (and (looking-at "\\(#\\sw+\\)") 0)
   ;; .custom, .func, .endfunc, .name, .backend
   (and (looking-at "\\(\\.\\(custom\\|func\\|endfunc\\|name\\|backend\\)\\)") 0)
   bc-basic-indent))

(defvar bc-mode-map
  (let ((map (make-sparse-keymap)))
    ; Auto-indent on : insertion and on newline
    (define-key map ":"    '(lambda () (interactive) (insert ":") (bc-indent-line)))
    (define-key map "\C-j" '(lambda () (interactive) (bc-indent-line) (newline-and-indent)))
    (define-key map "\C-m" '(lambda () (interactive) (bc-indent-line) (newline-and-indent)))
    map)
  "Keymap for bc mode.")

(defun bc-mode ()
  "Major mode for editing MutekH-bytecode files."
  (interactive)

  (kill-all-local-variables)
  (set-syntax-table bc-syntax-table)

  (set (make-local-variable 'font-lock-defaults) '(bc-font-lock-keywords))
  (use-local-map bc-mode-map)

  (make-local-variable 'indent-line-function)
  (setq indent-line-function 'bc-indent-line)

  (setq major-mode 'bc-mode)
  (setq mode-name "bc")

  (setq indent-tabs-mode nil)

  (run-hooks 'bc-mode-hook))

(provide 'bc-mode)
